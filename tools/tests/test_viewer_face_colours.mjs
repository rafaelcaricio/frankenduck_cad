import assert from 'node:assert/strict';
import test from 'node:test';
import { buildFaceColorData } from '../viewer/face_colours.mjs';

function fixture() {
  return {
    positions: new Float32Array(18),
    indices: new Uint32Array([2, 0, 1, 5, 3, 4]),
    faceRanges: [
      { ord: 1, indexStart: 0, indexCount: 3, color: [0.75, 0, 0, 1] },
      { ord: 2, indexStart: 3, indexCount: 3, color: [0, 0.5, 0, 1] },
    ],
  };
}

test('one component retains two face colours across indexed face ranges', () => {
  const component = fixture();
  const positions = component.positions.slice();
  const indices = component.indices.slice();
  const data = buildFaceColorData(component);
  assert.equal(data.hasFaceColors, true);
  assert.deepEqual(Array.from(data.colors), [
    0.75, 0, 0, 0.75, 0, 0, 0.75, 0, 0,
    0, 0.5, 0, 0, 0.5, 0, 0, 0.5, 0,
  ]);
  assert.deepEqual(component.positions, positions);
  assert.deepEqual(component.indices, indices);
});

test('face colours take precedence over the uniform part colour', () => {
  const data = buildFaceColorData(fixture(), [0, 0, 1, 1]);
  assert.deepEqual(Array.from(data.colors.slice(0, 3)), [0.75, 0, 0]);
});

test('uncoloured faces inherit the part colour when another face is coloured', () => {
  const component = fixture();
  component.faceRanges[1].color = null;
  const data = buildFaceColorData(component, [0.25, 0.25, 0.25, 1]);
  assert.deepEqual(Array.from(data.colors.slice(9)), Array(9).fill(0.25));
});

test('uniform and uncoloured parts keep the existing renderer path', () => {
  const component = fixture();
  for (const face of component.faceRanges) face.color = null;
  for (const color of [null, [1, 0, 0, 1]]) {
    const data = buildFaceColorData(component, color);
    assert.equal(data.hasFaceColors, false);
    assert.equal(data.colors.length, 0);
  }
});

test('invalid colour metadata does not inject non-finite vertex channels', () => {
  const component = fixture();
  component.faceRanges[0].color = [NaN, 1, 1];
  const data = buildFaceColorData(component, [0.25, 0.25, 0.25]);
  assert.deepEqual(Array.from(data.colors.slice(0, 9)), Array(9).fill(0.25));
  assert.ok(Array.from(data.colors).every(Number.isFinite));
});
