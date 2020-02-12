import numpy as np

actuation_saturation = 5

error = np.array([0.2, -0.6, 0.8])

error_integral = [2,5,7]

actuation = np.array([3, 7, 8])

actuation_clipped = np.clip(actuation, -actuation_saturation, actuation_saturation)

not_equal = np.not_equal(actuation, actuation_clipped)
print(not_equal)

dot = error * actuation
# print(dot)


greater = np.greater(dot, np.array([0,0,0]))
print(greater)


logical_and = np.logical_and(not_equal, greater)
print(logical_and)


a = np.array([3,4,5])

b = np.array([1,1,1])

a += b*np.invert(logical_and)

print(a)