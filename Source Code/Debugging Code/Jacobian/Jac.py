import sympy

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

x = -65*(((-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*cos(d) - sin(b)*sin(d)*cos(a))*cos(e) + (-sin(a)*cos(c) - sin(c)*cos(a)*cos(b))*sin(e))*sin(f) + 65*(-(-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*sin(d) - sin(b)*cos(a)*cos(d))*cos(f) - 350*(-sin(a)*sin(c) + cos(a)*cos(b)*cos(c))*sin(d) - 350*sin(b)*cos(a)*cos(d) - 400*sin(b)*cos(a) + 100*cos(a)
y = -65*(((sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*cos(d) - sin(a)*sin(b)*sin(d))*cos(e) + (-sin(a)*sin(c)*cos(b) + cos(a)*cos(c))*sin(e))*sin(f) + 65*(-(sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*sin(d) - sin(a)*sin(b)*cos(d))*cos(f) - 350*(sin(a)*cos(b)*cos(c) + sin(c)*cos(a))*sin(d) - 350*sin(a)*sin(b)*cos(d) - 400*sin(a)*sin(b) + 100*sin(a)
z = -65*((sin(b)*cos(c)*cos(d) + sin(d)*cos(b))*cos(e) - sin(b)*sin(c)*sin(e))*sin(f) + 65*(-sin(b)*sin(d)*cos(c) + cos(b)*cos(d))*cos(f) - 350*sin(b)*sin(d)*cos(c) + 350*cos(b)*cos(d) + 400*cos(b) + 320


print('J11 = ', sympy.diff(x, a))
print(end='\n')

print('J12 = ', sympy.diff(x, b))
print(end='\n')

print('J13 = ', sympy.diff(x, c))
print(end='\n')

print('J14= ', sympy.diff(x, d))
print(end='\n')

print('J15 = ', sympy.diff(x, e))
print(end='\n')

print('J16 = ', sympy.diff(x, f))
print(end='\n')

print('J17 = ', sympy.diff(x, g))
print(end='\n')


print('J21 = ', sympy.diff(y, a))
print(end='\n')

print('J22 = ', sympy.diff(y, b))
print(end='\n')

print('J23 = ', sympy.diff(y, c))
print(end='\n')

print('J24= ', sympy.diff(y, d))
print(end='\n')

print('J25 = ', sympy.diff(y, e))
print(end='\n')

print('J26 = ', sympy.diff(y, f))
print(end='\n')

print('J27 = ', sympy.diff(y, g))
print(end='\n')


print('J31 = ', sympy.diff(z, a))
print(end='\n')

print('J32 = ', sympy.diff(z, b))
print(end='\n')

print('J33 = ', sympy.diff(z, c))
print(end='\n')

print('J34= ', sympy.diff(z, d))
print(end='\n')

print('J35 = ', sympy.diff(z, e))
print(end='\n')

print('J36 = ', sympy.diff(z, f))
print(end='\n')

print('J37 = ', sympy.diff(z, g))
print(end='\n')


