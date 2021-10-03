import sympy
from sympy.matrices import Matrix
a = sympy.Symbol('a')
b = sympy.Symbol('b')
c = sympy.Symbol('c')
d = sympy.Symbol('d')
e = sympy.Symbol('e')
f = sympy.Symbol('f')
g = sympy.Symbol('g')

def cos(m):
    res = sympy.cos(m)
    return res

def sin(n):
    res = sympy.sin(n)
    return res

T1 = Matrix(4,4,[cos(a), 0, sin(a), 100*cos(a),
                 sin(a), 0, -cos(a), 100*sin(a),
                 0, 1, 0, 320,
                 0, 0, 0, 1])
T2 = Matrix(4,4,[cos(b), 0, -sin(b), 0,
                 sin(b), 0, cos(b), 0,
                 0, -1, 0, 0,
                 0, 0, 0, 1])

T3 = Matrix(4,4,[cos(c), 0, sin(c), 0,
                 sin(c), 0, -cos(c), 0,
                 0, 1, 0, 400,
                 0, 0, 0, 1])

T4 = Matrix(4,4,[cos(d), 0, -sin(d), 0,
                 sin(d), 0, cos(d), 0,
                 0, -1, 0, 0,
                 0, 0, 0, 1])

T5 = Matrix(4,4,[cos(e), 0, sin(e), 0,
                 sin(e), 0, -cos(e), 0,
                 0, 1, 0, 350,
                 0, 0, 0, 1])

T6 = Matrix(4,4,[cos(f), 0, -sin(f), 0,
                 sin(f), 0, cos(f), 0,
                 0, -1, 0, 0,
                 0, 0, 0, 1])

T7 = Matrix(4,4,[cos(g), -sin(g), 0, 0,
                 sin(g), cos(g), 0, 0,
                 0, 0, 1, 65,
                 0, 0, 0, 1])
T = T1*T2*T3*T4*T5*T6*T7


print(T[0,3])
print(end='\n')


print(T[1,3])
print(end='\n')


print(T[2,3])
print(end='\n')


print(T[3,3])
print(end='\n')

