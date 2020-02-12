import numpy as np
from sklearn.linear_model import LinearRegression

h = np.array([1.0, 3.0, 5.0, 7.0]).reshape((-1,1))

l = np.array([0.1, 1.0, 1.9, 2.8])
w = np.array([0.7, 2.8, 4.9, 7.0])

model_l = LinearRegression()
model_w = LinearRegression()

print(h)
print(l)
print(w)

model_l.fit(h, l)
model_w.fit(h, w)

r_sq_l = model_l.score(h, l)
r_sq_w = model_w.score(h, w)

print('coefficient of determination l:', r_sq_l)
print('coefficient of determination w:', r_sq_w)

print('intercept l:', model_l.intercept_)
print('slope l:', model_l.coef_)

print('intercept w:', model_w.intercept_)
print('slope w:', model_w.coef_)