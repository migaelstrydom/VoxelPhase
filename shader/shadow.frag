// Fragment stage for the sun shadow map.
//
// The pass has no colour attachment, so this writes nothing and exists only
// because a graphics pipeline without a fragment stage is a portability risk on
// MoltenVK. Depth is written by fixed function either way.

#version 450

void main() {
}
