import pytest
import numpy as np

from rocketpy.tools import Vector

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_constructor(vector_components):
    vector = Vector(vector_components)
    assert vector.components == vector_components

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_get_item(vector_components):
    vector = Vector(vector_components)
    for i in range(3):
        assert vector[i] = vector_components[i]

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_len(vector_components):
    vector = Vector(vector_components)
    assert len(vector) == 3

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_x_y_z(vector_components):
    vector = Vector(vector_components)
    assert vector.x == vector_components[0]
    assert vector.y == vector_components[1]
    assert vector.z == vector_components[2]

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_unit_vector(vector_components):
    vector = Vector(vector_components)
    unit_vector = vector.unit_vector
    assert abs(unit_vector) == 1
    assert unit_vector * vector == 0

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_cross_matrix(vector_components):
    vector = Vector(vector_components)
    cross_matrix = vector.cross_matrix
    assert cross_matrix.transpose == -cross_matrix
    assert cross_matrix.trace == 0
    assert cross_matrix @ vector == 0
    assert cross_matrix @ Vector.i == vector * Vector.i
    assert cross_matrix @ Vector.j == vector * Vector.j
    assert cross_matrix @ Vector.k == vector * Vector.k

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_abs(vector_components):
    vector = Vector(vector_components)
    vector_magnitude = abs(vector)
    assert vector_magnitude == sum([i**2 for i in vector_components])**0.5

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_neg(vector_components):
    vector = Vector(vector_components)
    neg_vector = Vector([-i for i in vector_components])
    assert vector == -vector

@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
@pytest.mark.parametrize("v_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_add(u_c, v_c):
    u, v = Vector(u_c), Vector(v_c)
    result = u + v
    assert result == Vector([i + j for i, j in zip(u_c, v_c)])

@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
@pytest.mark.parametrize("v_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_radd(u_c, v_c):
    u, v = u_c, Vector(v_c)
    result = u + v
    assert result == Vector([i + j for i, j in zip(u_c, v_c)])

@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
@pytest.mark.parametrize("v_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_sub(u_c, v_c):
    u, v = Vector(u_c), Vector(v_c)
    result = u - v
    assert result == Vector([i - j for i, j in zip(u_c, v_c)])

@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
@pytest.mark.parametrize("v_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_rsub(u_c, v_c):
    u, v = u_c, Vector(v_c)
    result = u - v
    assert result == Vector([i - j for i, j in zip(u_c, v_c)])


@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
@pytest.mark.parametrize("v_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_mul_cross(u_c, v_c):
    u, v = Vector(u_c), Vector(v_c)
    result = u * v
    assert result == np.cross(u_c, v_c)

@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
@pytest.mark.parametrize("v_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_rmul_cross(u_c, v_c):
    u, v = u_c, Vector(v_c)
    result = u * v
    assert result == np.cross(u_c, v_c)

@pytest.mark.parametrize("k", [-1, 0, 1, np.pi, -np.inf, np.inf])
@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_mul_scalar(k, u_c):
    u = Vector(u_c)
    result = k * u
    assert result == Vector([k*i for i in u_c])

@pytest.mark.parametrize("k", [-1, 0, 1, np.pi, -np.inf, np.inf])
@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_rmul_scalar(k, u_c):
    u = Vector(u_c)
    result = u * k
    assert result == Vector([k*i for i in u_c])

@pytest.mark.parametrize("k", [-1, 0, 1, np.pi, -np.inf, np.inf])
@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_truediv(k, u_c):
    u = Vector(u_c)
    result = u/k
    assert result == Vector([k*i for i in u_c])

@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
@pytest.mark.parametrize("v_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_matmul(u_c, v_c):
    u, v = u_c, Vector(v_c)
    result = u @ v
    assert result == np.dot(u_c, v_c)

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_eq(vector_components):
    u, v = Vector(vector_components), Vector(vector_components)
    assert u == vector_components
    assert u == v
    assert (u == 2*v) == False

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_neq(vector_components):
    u, v = Vector(vector_components), 2*Vector(vector_components)
    assert u != v
    assert v != vector_components
    assert (u != u) == False

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_is_parallel_to(vector_components):
    u = Vector(vector_components)
    v = 2*Vector(vector_components)
    w = u - Vector.i()
    assert u.is_parallel_to(v) == True
    assert u.is_parallel_to(w) == False

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_is_perpendicular_to(vector_components):
    u = Vector(vector_components)
    v = u - Vector.i()
    projection = u.proj(v)
    projection_vector = projection*v.unit_vector
    w = u - projection_vector
    assert u.is_perpendicular_to(2*u) == False
    assert w.is_perpendicular_to(v) == True

@pytest.mark.parametrize("operation", [lambda i: i**2, lambda i: 1/i])
@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 1j]])
def test_vector_element_wise(u_c, operation):
    u = Vector(u_c)
    vector = u.element_wise(operation)
    assert vector ==  Vector([operation(i) for i in range(3)])

@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
@pytest.mark.parametrize("v_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_dot(u_c, v_c):
    u, v = u_c, Vector(v_c)
    result = u.dot(v)
    assert result == np.dot(u_c, v_c)

@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
@pytest.mark.parametrize("v_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_cross(u_c, v_c):
    u, v = Vector(u_c), Vector(v_c)
    result = u.cross(v)
    assert result == np.cross(u_c, v_c)

@pytest.mark.parametrize("u_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
@pytest.mark.parametrize("v_c", [[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_proj(u_c, v_c):
    u, v = Vector(u_c), Vector(v_c)
    projection = u.proj(v)
    projection_vector = projection*v.unit_vector
    assert (u - projection_vector) @ v == 0

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_str(vector_components):
    vector = Vector(vector_components)
    assert eval("Vector(" + str(vector) + ")") == vector

@pytest.mark.parametrize([[1, 2, 3], [-np.pi, np.inf, np.e], [3*1j, -2j, 0j]])
def test_vector_str(vector_components):
    vector = Vector(vector_components)
    assert eval(repr(vector).replace("(", "((").replace(")", "))")) == vector

def test_vector_zeros():
    assert Vector.zeros() == [0, 0, 0]

def test_vector_i():
    assert Vector.i() == [1, 0, 0]

def test_vector_j():
    assert Vector.j() == [0, 1, 0]

def test_vector_k():
    assert Vector.k() == [0, 0, 1]

