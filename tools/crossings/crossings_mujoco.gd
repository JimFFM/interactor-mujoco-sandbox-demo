extends SceneTree
## Crossings via the MuJoCo guest: each stroke polyline becomes one freejointed
## body of capsule geoms (radius = proximity/2, zero gravity), so a contact
## between geoms of different stroke bodies is a crossing at contact.pos. Nearby
## contacts from one crossing are coalesced within an epsilon.
##
## Two fixtures: a simple golden pair with known positions, and the CASSIE
## crossing_split fixture (three overshooting strokes, guest/curvenet checks.cpp)
## whose three pairwise crossings are the nine-node topology that check asserts.
## Negative controls move a stroke apart so a crossing drops. A failed load is a
## FAIL, never a skip.

func _seg(a: Vector3, b: Vector3, n: int) -> Array:
	var pts := []
	for i in range(n + 1):
		pts.push_back(a.lerp(b, float(i) / float(n)))
	return pts

func _mjcf(strokes: Array, r: float) -> String:
	var s := "<mujoco><option gravity='0 0 0'/><worldbody>"
	for stroke in strokes:
		s += "<body pos='0 0 0'><freejoint/>"
		var pts: Array = stroke
		for i in range(pts.size() - 1):
			var a: Vector3 = pts[i]; var b: Vector3 = pts[i + 1]
			s += "<geom type='capsule' size='%f' fromto='%f %f %f %f %f %f'/>" % [r, a.x, a.y, a.z, b.x, b.y, b.z]
		s += "</body>"
	s += "</worldbody></mujoco>"
	return s

func _geom_to_stroke(strokes: Array) -> PackedInt32Array:
	var m := PackedInt32Array()
	for si in strokes.size():
		for _i in range((strokes[si] as Array).size() - 1):
			m.push_back(si)
	return m

# Inter-stroke contacts coalesced into distinct crossing points within eps.
func _crossings(sb, strokes: Array, r: float, eps: float) -> Array:
	if not sb.vmcall("mjc_load_xml", _mjcf(strokes, r).to_utf8_buffer()):
		return [null]
	var g2s := _geom_to_stroke(strokes)
	var raw := sb.vmcall("mjc_contacts") as PackedFloat64Array
	var reps := []
	for k in range(raw.size() / 6):
		var g0 := int(raw[k * 6 + 0]); var g1 := int(raw[k * 6 + 1])
		if g0 < 0 or g1 < 0 or g0 >= g2s.size() or g1 >= g2s.size(): continue
		if g2s[g0] == g2s[g1]: continue
		var pos := Vector3(raw[k * 6 + 2], raw[k * 6 + 3], raw[k * 6 + 4])
		var merged := false
		for rp in reps:
			if pos.distance_to(rp) < eps: merged = true; break
		if not merged: reps.push_back(pos)
	return reps

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
	if sb == null: printerr("FAIL crossings_mujoco: Sandbox class missing (extension not loaded)"); quit(1); return
	sb.set("program", load("res://plans/mujoco.elf"))
	sb.set_memory_max(1024); sb.set_allocations_max(1 << 21); sb.set_unboxed_arguments(true)

	# Fixture 1: simple golden. A horizontal crossed by two verticals -> 2 crossings.
	var A := [Vector3(-0.5, 0, 0), Vector3(0.5, 0, 0)]
	var B := [Vector3(0, -0.5, 0), Vector3(0, 0.5, 0)]
	var C := [Vector3(0.3, -0.5, 0), Vector3(0.3, 0.5, 0)]
	var g1 := _crossings(sb, [A, B, C], 0.05, 0.02)
	if g1 == [null]: printerr("FAIL crossings_mujoco: golden load failed"); quit(1); return
	var e1 := [Vector3(0, 0, 0), Vector3(0.3, 0, 0)]
	var ok1 := _all_near(g1, e1, 0.05)

	# Fixture 2: the CASSIE crossing_split strokes (checks.cpp:300-306), tessellated
	# 16, proximity 0.02 -> capsule radius 0.01. Three pairwise crossings, the nine
	# nodes the CASSIE check asserts.
	var s0 := _seg(Vector3(-1.2, 0, 0), Vector3(1.2, 0, 0), 16)
	var s1 := _seg(Vector3(1.1, -0.3, 0), Vector3(-0.2, 1.8, 0), 16)
	var s2 := _seg(Vector3(0.2, 1.8, 0), Vector3(-1.1, -0.3, 0), 16)
	var g2 := _crossings(sb, [s0, s1, s2], 0.01, 0.05)
	if g2 == [null]: printerr("FAIL crossings_mujoco: cassie fixture load failed"); quit(1); return
	var e2 := [Vector3(0.9143, 0, 0), Vector3(-0.9143, 0, 0), Vector3(0, 1.4769, 0)]
	var ok2 := _all_near(g2, e2, 0.05)

	# Negative control: shift s2 far in x so it crosses neither s0 nor s1 -> 1 crossing (s0 x s1).
	var s2far := _seg(Vector3(5.2, 1.8, 0), Vector3(3.9, -0.3, 0), 16)
	var ctrl := _crossings(sb, [s0, s1, s2far], 0.01, 0.05)
	var ok_ctrl := ctrl.size() == 1

	if ok1 and ok2 and ok_ctrl:
		print("DONE crossings_mujoco: golden %d/2, CASSIE fixture %d/3 at analytic crossings, control %d/1; guest mujoco.elf" % [g1.size(), g2.size(), ctrl.size()])
		quit(0)
	else:
		printerr("FAIL crossings_mujoco: golden=%d(ok=%s) cassie=%d(ok=%s) control=%d(ok=%s)" % [g1.size(), str(ok1), g2.size(), str(ok2), ctrl.size(), str(ok_ctrl)])
		quit(1)
