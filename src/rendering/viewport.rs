//! The dynamic viewport and scissor every pipeline here takes: the whole
//! target, depth 0 to 1.

use ash::vk;

/// Set `cb`'s viewport and scissor to cover all of `extent`.
pub fn set_full_viewport(device: &ash::Device, cb: vk::CommandBuffer, extent: vk::Extent2D) {
    let viewport = vk::Viewport {
        x: 0.0,
        y: 0.0,
        width: extent.width as f32,
        height: extent.height as f32,
        min_depth: 0.0,
        max_depth: 1.0,
    };
    unsafe {
        device.cmd_set_viewport(cb, 0, &[viewport]);
        device.cmd_set_scissor(cb, 0, &[extent.into()]);
    }
}
