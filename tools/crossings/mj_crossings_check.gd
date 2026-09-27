extends SceneTree
## Validates the guest's mj_crossings entry end-to-end: host passes strokes as a
## flat points array + per-stroke counts, the guest builds the capsule model,
## collides, coalesces and returns crossing x,y,z triples. Fixture is CASSIE's
## crossing_split (three overshooting strokes) with its three known crossings.

func _seg(a: Vector3, b: Vector3, n: int) -> Array:
	var p := []
	for i in range(n + 1): p.push_back(a.lerp(b, float(i) / float(n)))
	return p

func _flat(strokes: Array) -> Array:
	var pts := PackedFloat32Array(); var cnt := PackedInt32Array()
	for stroke in strokes:
		cnt.push_back((stroke as Array).size())
		for v in stroke: pts.append_array([v.x, v.y, v.z])
	return [pts, cnt]

func _cross(sb, strokes: Array) -> Array:
	var fc := _flat(strokes)
	var raw := sb.vmcall("mj_crossings", fc[0], fc[1], 0.02) as PackedFloat64Array
	var out := []
	for k in range(raw.size() / 3):
		out.push_back(Vector3(raw[k * 3], raw[k * 3 + 1], raw[k * 3 + 2]))
	return out

func _all_near(got: Array, expect: Array, tol: float) -> bool:
	if got.size() != expect.size(): return false
	for e in expect:
		var hit := false
		for g in got:
			if (g as Vector3).distance_to(e) < tol: hit = true
		if not hit: return false
	return true

func _init() -> void:
	var sb = ClassDB.instantiate("Sandbox")
	if sb == null: printerr("FAIL mj_crossings: Sandbox class missing"); quit(1); return
	sb.set("program", load("res://plans/mujoco.elf"))
	sb.set_memory_max(1024); sb.set_allocations_max(1 << 21); sb.set_unboxed_arguments(true)
	var s0 := _seg(Vector3(-1.2, 0, 0), Vector3(1.2, 0, 0), 16)
	var s1 := _seg(Vector3(1.1, -0.3, 0), Vector3(-0.2, 1.8, 0), 16)
	var s2 := _seg(Vector3(0.2, 1.8, 0), Vector3(-1.1, -0.3, 0), 16)
	var got := _cross(sb, [s0, s1, s2])
	var expect := [Vector3(0.9143, 0, 0), Vector3(-0.9143, 0, 0), Vector3(0, 1.4769, 0)]
	var ok := _all_near(got, expect, 0.05)
	# negative control: shift s2 far so it crosses neither -> one crossing
	var s2far := _seg(Vector3(5.2, 1.8, 0), Vector3(3.9, -0.3, 0), 16)
	var ctrl := _cross(sb, [s0, s1, s2far])
	if ok and ctrl.size() == 1:
		print("DONE mj_crossings: %d/3 crossings at analytic points; control %d/1; guest entry" % [got.size(), ctrl.size()])
		quit(0)
	else:
		printerr("FAIL mj_crossings: got=%d(ok=%s) control=%d" % [got.size(), str(ok), ctrl.size()]); quit(1)
