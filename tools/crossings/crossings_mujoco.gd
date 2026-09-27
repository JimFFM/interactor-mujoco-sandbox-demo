extends SceneTree
## Crossings via the MuJoCo guest: each stroke polyline becomes one freejointed
## body of capsule geoms (radius = proximity_threshold/2, zero gravity). A
## contact between geoms of different stroke bodies is a crossing at contact.pos.
## Golden fixture with a known crossing count/positions; a negative control moves
## a stroke apart so its crossing drops. A failed load is a FAIL, never a skip.

const R := 0.05   # capsule radius; two strokes cross when centerlines pass within 2R

func _mjcf(strokes: Array) -> String:
	var s := "<mujoco><option gravity='0 0 0'/><worldbody>"
	for stroke in strokes:
		s += "<body pos='0 0 0'><freejoint/>"
		var pts: Array = stroke
		for i in range(pts.size() - 1):
			var a: Vector3 = pts[i]; var b: Vector3 = pts[i + 1]
			s += "<geom type='capsule' size='%f' fromto='%f %f %f %f %f %f'/>" % [R, a.x, a.y, a.z, b.x, b.y, b.z]
		s += "</body>"
	s += "</worldbody></mujoco>"
	return s

# geom index -> stroke index, in MJCF emission order (geoms are numbered per body, in order).
func _geom_to_stroke(strokes: Array) -> PackedInt32Array:
	var map := PackedInt32Array()
	for si in strokes.size():
		for _i in range((strokes[si] as Array).size() - 1):
			map.push_back(si)
	return map

func _crossings(sb, strokes: Array) -> Array:
	if not sb.vmcall("mjc_load_xml", _mjcf(strokes).to_utf8_buffer()):
		return [null]   # load failure sentinel
	var g2s := _geom_to_stroke(strokes)
	var raw := sb.vmcall("mjc_contacts") as PackedFloat64Array
	var out := []   # [stroke_a, stroke_b, Vector3 pos] for inter-stroke contacts only
	for k in range(raw.size() / 6):
		var g0 := int(raw[k * 6 + 0]); var g1 := int(raw[k * 6 + 1])
		if g0 < 0 or g1 < 0 or g0 >= g2s.size() or g1 >= g2s.size(): continue
		if g2s[g0] == g2s[g1]: continue   # same stroke, not a crossing
		out.push_back([g2s[g0], g2s[g1], Vector3(raw[k * 6 + 2], raw[k * 6 + 3], raw[k * 6 + 4])])
	return out

func _init() -> void:
	var sb = ClassDB.instantiate("Sandbox")
	if sb == null: printerr("FAIL crossings_mujoco: Sandbox class missing (extension not loaded)"); quit(1); return
	sb.set("program", load("res://plans/mujoco.elf"))
	sb.set_memory_max(1024); sb.set_allocations_max(1 << 21); sb.set_unboxed_arguments(true)

	# Golden fixture: A horizontal; B, C vertical. A crosses B at origin and C at
	# x=0.3; B and C are parallel and do not cross. Known: 2 crossings.
	var A := [Vector3(-0.5, 0, 0), Vector3(0.5, 0, 0)]
	var B := [Vector3(0, -0.5, 0), Vector3(0, 0.5, 0)]
	var C := [Vector3(0.3, -0.5, 0), Vector3(0.3, 0.5, 0)]
	var got := _crossings(sb, [A, B, C])
	if got == [null]: printerr("FAIL crossings_mujoco: model load failed"); quit(1); return
	var expect := [Vector3(0, 0, 0), Vector3(0.3, 0, 0)]
	var ok_count := got.size() == expect.size()
	var ok_pos := ok_count
	for c in got:
		var pos: Vector3 = c[2]
		var near := false
		for e in expect:
			if pos.distance_to(e) < R: near = true
		ok_pos = ok_pos and near

	# Negative control: move C far in x so it no longer crosses A -> 1 crossing.
	var Cfar := [Vector3(3.0, -0.5, 0), Vector3(3.0, 0.5, 0)]
	var ctrl := _crossings(sb, [A, B, Cfar])
	var ok_ctrl := ctrl.size() == 1

	if ok_count and ok_pos and ok_ctrl:
		print("DONE crossings_mujoco: %d/%d crossings at known positions; control drops to %d; device=guest mujoco.elf" % [got.size(), expect.size(), ctrl.size()])
		quit(0)
	else:
		printerr("FAIL crossings_mujoco: count=%d(exp %d) pos_ok=%s control=%d(exp 1)" % [got.size(), expect.size(), str(ok_pos), ctrl.size()])
		quit(1)
