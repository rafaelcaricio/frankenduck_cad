/** Preserve intrinsic SURF face colours when displaying a single CAD part.
 *
 * SURF and tessellation face ranges carry linear RGBA. Three.js vertex colours
 * use linear RGB too, so copy the channels without a second gamma conversion.
 * Vertices are separate at face boundaries; filling the indexed face ranges
 * changes appearance only, never positions, indices or component identity.
 */
export function buildFaceColorData(component, partColor = null) {
  const ranges = component.faceRanges || [];
  const valid = (color) => Array.isArray(color)
    && color.length >= 3
    && color.slice(0, 3).every(Number.isFinite);
  const hasFaceColors = ranges.some((face) => valid(face.color));
  if (!hasFaceColors) {
    return { colors: new Float32Array(), hasFaceColors: false };
  }

  const colors = new Float32Array(component.positions.length);
  const fallback = valid(partColor) ? partColor : [1, 1, 1];
  const clamp = (channel) => Math.max(0, Math.min(1, channel));
  for (let offset = 0; offset < colors.length; offset += 3) {
    colors.set(fallback.slice(0, 3).map(clamp), offset);
  }
  for (const face of ranges) {
    if (!valid(face.color)) continue;
    const rgb = face.color.slice(0, 3).map(clamp);
    const end = face.indexStart + face.indexCount;
    for (let index = face.indexStart; index < end; index += 1) {
      colors.set(rgb, component.indices[index] * 3);
    }
  }
  return { colors, hasFaceColors: true };
}
