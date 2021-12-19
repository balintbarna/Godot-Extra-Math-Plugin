extends Reference
class_name ExtraMath


const EPSILON = 0.00001
const EPSILON2 = (EPSILON * EPSILON)
const SQRTHALF = 0.7071067811865475244008443621048490
const SQRT2 = 1.4142135623730950488016887242
const LN2 = 0.6931471805599453094172321215
const EULER = 2.7182818284590452353602874714
const DEFAULT_EULER_ORDER = "yxz"


# Convert `int` microseconds to `float` seconds.
static func usec_to_sec(usec: int):
    return usec / 1000000.0


# Convert `float` seconds to `int` microseconds.
static func sec_to_usec(sec: float):
    return int(sec * 1000000)


# Returns the rotation in axis-angle representation, which rotates the direction of the `from` vector to the direction of the `to` vector.
static func get_vectors_rotation(from: Vector3, to: Vector3) -> Vector3:
    var dot = from.dot(to) # dot = |a|×|b|× cos(fi)
    var cross = from.cross(to) # cross = |a|×|b|× sin(fi) × n
    var length_product = from.length() * to.length()
    if length_product == 0:
        push_error("Cannot calculate rotation from or to ZERO vector.")
        return Vector3.ZERO
    var cos_fi = dot / length_product
    var sin_fi = cross.length() / length_product
    var fi = atan2(sin_fi, cos_fi)
    return cross.normalized() * fi


# Same as `quat2axis_angle` but for Basises
static func basis2axis_angle(b: Basis):
    return quat2axis_angle(Quat(b))


# Easy way to convert your quaternion rotation to axis-angle representation. The vector length is equal to the amount of rotation in radians.
static func quat2axis_angle(q: Quat):
    var v = Vector3(q.x, q.y, q.z)
    var angle = 2 * atan2(v.length(), q.w)
    return v.normalized() * angle


# Same as `basis2euler` but with a quaternion
static func quat2euler(q: Quat, order = DEFAULT_EULER_ORDER) -> Vector3:
    return basis2euler(Basis(q), order)


# Converts a basis to an Euler representation in the specified rotation order, default order is "yxz".
static func basis2euler(b: Basis, order = DEFAULT_EULER_ORDER) -> Vector3:
    # Based on https://en.wikipedia.org/wiki/Euler_angles#Rotation_matrix
    var euler = Vector3()
    if order == "xzy":
        # rot =  cz*cy             -sy             cz*sy
        #        sx*sy+cx*cy*sz    cx*cz           cx*sz*sy-cy*sx
        #        cy*sx*sz          cz*sx           cx*cy+sx*sz*sy
        euler.x = atan2(b.y.z, b.y.y)
        euler.y = atan2(b.z.x, b.x.x)
        var sz = -b.y.x
        var cz = b.y.y / cos(euler.x)
        euler.z = atan2(sz, cz)
    elif order == "xyz":
        # rot =  cy*cz          -cy*sz           sy
        #        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
        #       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
        euler.x = atan2(-b.z.y, b.z.z)
        euler.z = atan2(-b.y.x, b.x.x)
        var sy = b.z.x
        var cy = b.x.x / cos(euler.z)
        euler.y = atan2(sy, cy)
    elif order == "yxz":
        # rot =  cy*cz+sy*sx*sz    cz*sy*sx-cy*sz        cx*sy
        #        cx*sz             cx*cz                 -sx
        #        cy*sx*sz-cz*sy    cy*cz*sx+sy*sz        cy*cx
        euler.y = atan2(b.z.x, b.z.z)
        euler.z = atan2(b.x.y, b.y.y)
        var sx = -b.z.y
        var cx = b.y.y / cos(euler.z)
        euler.x = atan2(sx, cx)
    elif order == "yzx":
        # rot =  cy*cz             sy*sx-cy*cx*sz     cx*sy+cy*sz*sx
        #        sz                cz*cx              -cz*sx
        #        -cz*sy            cy*sx+cx*sy*sz     cy*cx-sy*sz*sx
        euler.x = atan2(-b.z.y, b.y.y)
        euler.y = atan2(-b.x.z, b.x.x)
        var sz = b.x.y
        var cz = b.y.y / cos(euler.x)
        euler.z = atan2(sz, cz)
    elif order == "zxy":
        # rot =  cz*cy-sz*sx*sy    -cx*sz                cz*sy+cy*sz*sx
        #        cy*sz+cz*sx*sy    cz*cx                 sz*sy-cz*cy*sx
        #        -cx*sy            sx                    cx*cy
        euler.y = atan2(-b.x.z, b.z.z)
        euler.z = atan2(-b.y.x, b.y.y)
        var sx = b.y.z
        var cx = b.y.y / cos(euler.z)
        euler.x = atan2(sx, cx)
    elif order == "zyx":
        # rot =  cz*cy             cz*sy*sx-cx*sz        sz*sx+cz*cx*cy
        #        cy*sz             cz*cx+sz*sy*sx        cx*sz*sy-cz*sx
        #        -sy               cy*sx                 cy*cx
        euler.x = atan2(b.y.z, b.z.z)
        euler.z = atan2(b.x.y, b.x.x)
        var sy = -b.x.z
        var cy = b.x.x / cos(euler.z)
        euler.y = atan2(sy, cy)
    else:
        push_error("Euler order must contain exactly one of X, Y, and Z axes in lowercase, e.g. 'xyz'. Proper Euler orders, such as 'zxz' are not supported.")
    return euler


# Same as `euler2basis` but converts to quaternion in one step
static func euler2quat(angles: Vector3, order = DEFAULT_EULER_ORDER) -> Quat:
        return Quat(euler2basis(angles, order))


# Calculate the rotation matrix from Euler representation in your specified order. The xyz values in `angles` vector denote rotation around the respective local axis. Default order is "yxz".
static func euler2basis(angles: Vector3, order = DEFAULT_EULER_ORDER) -> Basis:
    if not (order.length() == 3 and "x" in order and "y" in order and "z" in order):
        push_error("Euler order must contain exactly one of X, Y, and Z axes in lowercase, e.g. 'xyz'. Proper Euler orders, such as 'zxz' are not supported.")
        return Basis()
    var rots = {
        "x": Basis(Vector3.RIGHT, angles.x),
        "y": Basis(Vector3.UP, angles.y),
        "z": Basis(Vector3.BACK, angles.z),
    }
    return rots[order[0]] * rots[order[1]] * rots[order[2]]
