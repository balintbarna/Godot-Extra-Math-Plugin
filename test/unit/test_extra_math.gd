extends "res://addons/gut/test.gd"


var vecs = []
var mats = []
func before_all():
    for __ in range(100):
        vecs.append(Vector3(
            rand_range(-5, 5),
            rand_range(-5, 5),
            rand_range(-5, 5)
        ))
    for v in vecs:
        mats.append(Basis(v.normalized(), v.length()))


func test_basis2euler_default():
    for b in mats:
        var euler = ExtraMath.basis2euler(b)
        var euler_built_in = b.get_euler()
        assert_vecs_almost_eq(euler, euler_built_in, ExtraMath.EPSILON)


func test_euler2basis_default():
    for b in mats:
        var euler = b.get_euler()
        var b_calc = ExtraMath.euler2basis(euler)
        assert_mats_almost_eq(b_calc, b, ExtraMath.EPSILON)


func test_basis2euler2basis_all_orders():
    var orders = ["xzy", "xyz", "yxz", "yzx", "zxy", "zyx"]
    for b in mats:
        for order in orders:
            var e_calc = ExtraMath.basis2euler(b, order)
            var b_calc = ExtraMath.euler2basis(e_calc, order)
            assert_mats_almost_eq(b_calc, b, ExtraMath.EPSILON)


func test_basis2axis_angle():
    for b in mats:
        var v = ExtraMath.basis2axis_angle(b)
        var b_calc = Basis(v.normalized(), v.length()) # by definition
        assert_mats_almost_eq(b_calc, b, ExtraMath.EPSILON)


func test_get_vectors_rotation():
    for __ in range(1000): # do 1000 random combinations
        var from = vecs[randi() % 100]
        var to = vecs[randi() % 100]
        var rot = ExtraMath.get_vectors_rotation(from, to)
        var to_calculated = from.rotated(rot.normalized(), rot.length())
        var to_calculated_with_basis = Basis(rot.normalized(), rot.length()) * from
        assert_vecs_almost_eq(to_calculated.normalized(), to.normalized(), ExtraMath.EPSILON)
        assert_vecs_almost_eq(to_calculated_with_basis.normalized(), to.normalized(), ExtraMath.EPSILON)


func assert_vecs_almost_eq(got, expected, max_err):
    var passing = (got-expected).length() < max_err
    var message = "got {}, expected {}".format([got, expected], "{}")
    assert_true(passing, message)


func assert_mats_almost_eq(got, expected, max_err):
    var passing = (got.inverse() * expected).get_euler().length() < max_err
    var message = "got {}, expected {}".format([got, expected], "{}")
    assert_true(passing, message)
