//! Endpoint enumeration and properties: IDs, friendly names, transport inference, physical
//! grouping by container ID, mix format / engine period, endpoint volume.

use std::time::Duration;

use windows::Win32::Devices::FunctionDiscovery::{
    PKEY_Device_ContainerId, PKEY_Device_EnumeratorName, PKEY_Device_FriendlyName,
    PKEY_Device_InstanceId,
};
use windows::Win32::Media::Audio::Endpoints::IAudioEndpointVolume;
use windows::Win32::Media::Audio::{
    eCapture, eConsole, eRender, EDataFlow, IAudioClient, IDeviceTopology, IMMDevice,
    IMMDeviceEnumerator, PKEY_AudioEndpoint_FormFactor, PKEY_AudioEngine_DeviceFormat,
    DEVICE_STATE, DEVICE_STATE_ACTIVE,
};
use windows::Win32::System::Com::{CLSCTX_ALL, STGM_READ};
use windows::Win32::UI::Shell::PropertiesSystem::IPropertyStore;
use windows_core::HSTRING;
use wye_platform::{DeviceId, DeviceInfo, DeviceMode, PlatformError, ReportedLatency, Transport};

use crate::classify::{
    self, classify_endpoint, is_hands_free_format, physical_group, EndpointClass, EndpointView,
    COMPUTER_CONTAINER,
};
use crate::com::{
    default_period_hns, hns_to_frames, os_err, prop_guid, prop_string, prop_u32, prop_wave_format,
    take_pwstr, MixFormat,
};
use crate::format::{RawWaveFormat, WaveFormat};

/// Everything Wye reads from one endpoint.
pub(crate) struct Endpoint {
    pub(crate) device: IMMDevice,
    pub(crate) id: String,
    pub(crate) name: String,
    pub(crate) class: EndpointClass,
    pub(crate) form_factor: i32,
    pub(crate) container: Option<u128>,
    pub(crate) group: Option<String>,
}

impl Endpoint {
    pub(crate) fn onboard(&self) -> bool {
        self.container == Some(COMPUTER_CONTAINER)
    }

    pub(crate) fn is_bluetooth(&self) -> bool {
        matches!(
            self.class.transport,
            Transport::Bluetooth | Transport::BluetoothLe
        )
    }
}

pub(crate) fn device_id(device: &IMMDevice) -> Result<String, PlatformError> {
    // SAFETY: returns a CoTaskMem-allocated string, taken over by `take_pwstr`.
    unsafe { device.GetId() }
        .map(take_pwstr)
        .map_err(|e| os_err("IMMDevice::GetId", &e))
}

pub(crate) fn open_store(device: &IMMDevice) -> Option<IPropertyStore> {
    // SAFETY: read-only property store.
    unsafe { device.OpenPropertyStore(STGM_READ) }.ok()
}

/// Opens an endpoint by its persisted ID.
pub(crate) fn get_device(
    en: &IMMDeviceEnumerator,
    id: &DeviceId,
) -> Result<IMMDevice, PlatformError> {
    let wide = HSTRING::from(id.0.as_str());
    // SAFETY: valid NUL-terminated ID string.
    unsafe { en.GetDevice(&wide) }.map_err(|e| {
        if e.code().0 == classify::hr::E_NOTFOUND {
            PlatformError::DeviceNotFound(id.clone())
        } else {
            os_err("IMMDeviceEnumerator::GetDevice", &e)
        }
    })
}

/// Instance ID of the device the endpoint's topology connects to, e.g. `USB\VID_…` or
/// `BTHENUM\{…}` (the endpoint itself is a software device created by MMDEVAPI).
fn topology_instance_id(en: &IMMDeviceEnumerator, device: &IMMDevice) -> Option<String> {
    // SAFETY: plain COM calls on valid interfaces.
    unsafe {
        let topology: IDeviceTopology = device.Activate(CLSCTX_ALL, None).ok()?;
        let connector = topology.GetConnector(0).ok()?;
        let connected = take_pwstr(connector.GetDeviceIdConnectedTo().ok()?);
        if connected.is_empty() {
            return None;
        }
        let filter = en.GetDevice(&HSTRING::from(connected.as_str())).ok()?;
        prop_string(&open_store(&filter)?, &PKEY_Device_InstanceId)
    }
}

/// Reads the properties of one endpoint.
pub(crate) fn describe(
    en: &IMMDeviceEnumerator,
    device: IMMDevice,
) -> Result<Endpoint, PlatformError> {
    let id = device_id(&device)?;
    let store = open_store(&device);
    let name = store
        .as_ref()
        .and_then(|s| prop_string(s, &PKEY_Device_FriendlyName))
        .unwrap_or_else(|| id.clone());
    let form_factor = store
        .as_ref()
        .and_then(|s| prop_u32(s, &PKEY_AudioEndpoint_FormFactor))
        .map(|v| v as i32)
        .unwrap_or(10);
    let container = store
        .as_ref()
        .and_then(|s| prop_guid(s, &PKEY_Device_ContainerId));
    let topo = topology_instance_id(en, &device);
    let enumerator = store
        .as_ref()
        .and_then(|s| prop_string(s, &PKEY_Device_EnumeratorName));
    let mut candidates: Vec<&str> = Vec::with_capacity(2);
    if let Some(t) = topo.as_deref() {
        candidates.push(t);
    }
    if let Some(e) = enumerator.as_deref() {
        candidates.push(e);
    }
    let class = classify_endpoint(&candidates, form_factor);
    Ok(Endpoint {
        device,
        id,
        name,
        class,
        form_factor,
        container,
        group: physical_group(container),
    })
}

/// All endpoints of `flow` in `state_mask`.
pub(crate) fn list_endpoints(
    en: &IMMDeviceEnumerator,
    flow: EDataFlow,
    state_mask: DEVICE_STATE,
) -> Result<Vec<Endpoint>, PlatformError> {
    // SAFETY: plain COM calls on valid interfaces.
    let collection = unsafe { en.EnumAudioEndpoints(flow, state_mask) }
        .map_err(|e| os_err("EnumAudioEndpoints", &e))?;
    // SAFETY: as above.
    let count = unsafe { collection.GetCount() }.map_err(|e| os_err("GetCount", &e))?;
    let mut out = Vec::with_capacity(count as usize);
    for i in 0..count {
        // SAFETY: index in range.
        let Ok(device) = (unsafe { collection.Item(i) }) else {
            continue;
        };
        if let Ok(ep) = describe(en, device) {
            out.push(ep);
        }
    }
    Ok(out)
}

/// Active render endpoints that Wye lists (hands-free twins of Stereo endpoints hidden).
pub(crate) fn visible_outputs(en: &IMMDeviceEnumerator) -> Result<Vec<Endpoint>, PlatformError> {
    let all = list_endpoints(en, eRender, DEVICE_STATE_ACTIVE)?;
    let views: Vec<EndpointView<'_>> = all
        .iter()
        .map(|e| EndpointView {
            class: e.class,
            group: e.group.as_deref(),
        })
        .collect();
    let keep: Vec<bool> = (0..all.len())
        .map(|i| classify::output_visible(&views, i))
        .collect();
    Ok(all
        .into_iter()
        .zip(keep)
        .filter_map(|(e, k)| k.then_some(e))
        .collect())
}

/// Mix format and default engine period of an endpoint (activates an audio client without
/// initializing a stream).
pub(crate) fn mix_info(device: &IMMDevice) -> Option<(WaveFormat, u32)> {
    // SAFETY: plain COM activation.
    let client: IAudioClient = unsafe { device.Activate(CLSCTX_ALL, None) }.ok()?;
    let mix = MixFormat::of(&client).ok()?;
    let period = hns_to_frames(default_period_hns(&client), mix.format.rate);
    Some((mix.format, period))
}

/// The engine's device format (`PKEY_AudioEngine_DeviceFormat`), used by the hands-free heuristic.
pub(crate) fn device_format(device: &IMMDevice) -> Option<RawWaveFormat> {
    prop_wave_format(&open_store(device)?, &PKEY_AudioEngine_DeviceFormat)
}

/// Builds the platform description of an endpoint.
pub(crate) fn device_info(ep: &Endpoint) -> DeviceInfo {
    let (rate, channels, period) = match mix_info(&ep.device) {
        Some((f, p)) => (f.rate, f.channels, p),
        None => (0, 0, 0),
    };
    let mode = if ep.is_bluetooth() && rate > 0 && is_hands_free_format(rate, channels) {
        DeviceMode::BluetoothHandsFree
    } else {
        DeviceMode::Normal
    };
    // The render stream buffer is two engine periods; the clock position already refers to the
    // frame leaving the device, so nothing is added after the timestamp.
    let buffer = if rate > 0 {
        Duration::from_secs_f64(2.0 * period as f64 / rate as f64)
    } else {
        Duration::ZERO
    };
    DeviceInfo {
        id: DeviceId(ep.id.clone()),
        name: ep.name.clone(),
        transport: ep.class.transport,
        nominal_rate: rate as f64,
        output_channels: channels,
        mode,
        reported_latency: ReportedLatency {
            after_timestamp: Duration::ZERO,
            total: buffer,
        },
        physical_group: ep.group.clone(),
        can_be_default: true,
        buffer_frames: period,
        // Detecting exclusive use requires opening a stream; reported as a stream error instead.
        busy: false,
    }
}

/// Current default render endpoint for the console role.
pub(crate) fn default_output_id(en: &IMMDeviceEnumerator) -> Option<String> {
    // SAFETY: plain COM call; fails with E_NOTFOUND when there is no output at all.
    let device = unsafe { en.GetDefaultAudioEndpoint(eRender, eConsole) }.ok()?;
    device_id(&device).ok()
}

/// Master volume scalar and mute state of an endpoint.
pub(crate) fn endpoint_volume(device: &IMMDevice) -> Option<(f32, bool)> {
    // SAFETY: plain COM calls.
    unsafe {
        let vol: IAudioEndpointVolume = device.Activate(CLSCTX_ALL, None).ok()?;
        let scalar = vol.GetMasterVolumeLevelScalar().ok()?;
        let muted = vol.GetMute().ok()?.as_bool();
        Some((scalar, muted))
    }
}

/// Containers of Bluetooth render endpoints in any state (a paired headset whose output is
/// currently inactive still shares its container with its hands-free microphone).
pub(crate) fn bluetooth_output_groups(en: &IMMDeviceEnumerator) -> Vec<String> {
    let all = list_endpoints(
        en,
        eRender,
        DEVICE_STATE(windows::Win32::Media::Audio::DEVICE_STATEMASK_ALL),
    )
    .unwrap_or_default();
    let mut groups: Vec<String> = all
        .into_iter()
        .filter(|e| e.is_bluetooth())
        .filter_map(|e| e.group)
        .collect();
    groups.sort();
    groups.dedup();
    groups
}

/// Active capture endpoints.
pub(crate) fn capture_endpoints(en: &IMMDeviceEnumerator) -> Result<Vec<Endpoint>, PlatformError> {
    list_endpoints(en, eCapture, DEVICE_STATE_ACTIVE)
}
