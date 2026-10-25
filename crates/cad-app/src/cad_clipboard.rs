//! Interoperable between instances of this app; plain geometry/common style, not native CAD records.
use crate::ViewerState;
use cad_core::geometry_edit::ClipboardItem;
use std::collections::BTreeSet;

const PREFIX: &str = "CADCAM-RUST-GEOMETRY/1\n";
const MAX_BYTES: usize = 64 * 1024 * 1024;
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Payload {
    pub base: [f64; 3],
    pub items: Vec<ClipboardItem>,
}
pub fn encode(
    state: &ViewerState,
    ids: &BTreeSet<u32>,
    base: [f64; 3],
) -> Result<(Payload, String), String> {
    if ids.is_empty() {
        return Err("请先选择要复制的图元".into());
    }
    let doc = state.drawing.as_ref().ok_or("请先打开图纸")?.document();
    let mut items = Vec::new();
    for id in ids {
        let e = doc
            .entity(cad_core::EntityId(*id))
            .ok_or("选择中有失效对象")?;
        let l = doc.layer(e.layer_id).ok_or("源图层不存在")?;
        items.push(ClipboardItem {
            geometry: e.geometry.clone(),
            layer: l.name.clone(),
            layer_color: l.color,
            color: e.color,
            reference: state.excluded.contains(id)
                || matches!(e.geometry, cad_core::Geometry::Text(_)),
        });
    }
    let data = Payload { base, items };
    let text = format!(
        "{PREFIX}{}",
        serde_json::to_string(&data).map_err(|e| e.to_string())?
    );
    if text.len() > MAX_BYTES {
        return Err("几何剪贴板超过 64 MiB，请减少选择".into());
    }
    Ok((data, text))
}
pub fn decode(text: &str) -> Result<Payload, String> {
    if text.len() > MAX_BYTES {
        return Err("剪贴板超过 64 MiB".into());
    }
    let data: Payload = serde_json::from_str(
        text.strip_prefix(PREFIX)
            .or_else(|| text.strip_prefix("CADCAM-RUST-GEOMETRY/1\r\n"))
            .ok_or("剪贴板不是本应用的几何格式")?,
    )
    .map_err(|e| e.to_string())?;
    if !data.base.iter().all(|v| v.is_finite())
        || data.items.is_empty()
        || data.items.len() > 50_000
    {
        return Err("剪贴板基点或图元数量无效".into());
    }
    Ok(data)
}
