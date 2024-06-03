import numpy as np
import matplotlib.pyplot as plt
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures

# Data
distance = np.array([0.3, 1, 2, 3, 5, 6]).reshape(-1, 1)
c_1 = np.array([6.70E-05, 3.15E-05, 1.14E-04, 5.52E-04, 1.93E-03, 3.71E-03])
c_2 = np.array([5.16E-05, 1.10E-05, 9.76E-05, 4.19E-04, 6.89E-04, 3.17E-03])
c_3 = np.array([1.84E-04, 2.51E-04, 2.41E-03, 1.09E-02, 2.05E-02, 6.35E-02])
c_4 = np.array([9.27E-04, 5.93E-03, 8.12E-02, 6.87E-02, 4.21E-02, 1.19E-01])
c_5 = np.array([6.86E-04, 3.21E-03, 4.90E-02, 1.25E-01, 8.90E-02, 1.27E-01])
c_6 = np.array([1.66E-04, 3.28E-04, 1.40E-03, 1.64E-03, 2.24E-03, 3.32E-03])

data = c_6

# interval from 0 to 6 with 0.1 step
x_axis = np.arange(0, 6, 0.1).reshape(-1, 1)

# Linear Regression
linear_model = LinearRegression()
linear_model.fit(distance, data)
data_linear_pred = linear_model.predict(x_axis)

# Polynomial Regression
poly_features = PolynomialFeatures(degree=2)
distance_poly = poly_features.fit_transform(distance)
x_axis_poly = poly_features.fit_transform(x_axis)
poly_model = LinearRegression()
poly_model.fit(distance_poly, data)
data_poly_pred_2 = poly_model.predict(x_axis_poly)

# Polynomial Regression
poly_features_3 = PolynomialFeatures(degree=3)
distance_poly_3 = poly_features_3.fit_transform(distance)
x_axis_poly_3 = poly_features_3.fit_transform(x_axis)
poly_model_3 = LinearRegression()
poly_model_3.fit(distance_poly_3, data)
data_poly_pred_3 = poly_model_3.predict(x_axis_poly_3)

# Plotting
plt.scatter(distance, data, color='black', label='Data')
plt.plot(x_axis, data_linear_pred, color='red', label='Linear')
plt.plot(x_axis, data_poly_pred_2, color='green', label='Quadratic')
plt.plot(x_axis, data_poly_pred_3, color='blue', label='Cubic')
plt.xlabel('[m]')
plt.ylabel('std')
plt.ylim([min(data)-0.0001, max(data)+0.0001])
plt.legend()
plt.show()

# print the parameters
print("Linear model: ", linear_model.coef_)
print("Quadratic model: ", poly_model.coef_)
print("Cubic model: ", poly_model_3.coef_)