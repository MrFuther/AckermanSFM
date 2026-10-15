import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import statsmodels.api as sm

# Provided data
data = {
    "Right Force": [0.3878221694, 0.3701028759, 0.3809773876, -0.06940022053, -0.07981375951,
                    0.6826796392, 0.570695861, 0.7666668538, 0.854920519, 1.040001205,
                    1.511066599, 1.520356544, 0.0667675668, 0.92266074, 0.8971543339],
    "Left Force": [0.3364206071, 0.3525565676, 0.4040820701, -0.009714025397, 0.1221088113,
                   0.6475282745, 0.8833401354, 0.8896044254, 0.9867595197, 1.113640968,
                   1.577907255, 1.587845038, 0.4214727754, 1.063949607, 1.021283407],
    "fnav": [2.020297578, 2.019881823, 2.017237103, 1.999926301, 2.062422058,
             2.020882885, 2.408166085, 2.453069179, 2.503155271, 2.360945159,
             2.259256897, 2.260893216, 2.223132012, 2.364537846, 2.302582847],
    "fstatis": [0.7242427764, 0.7226594434, 0.7850594578, -0.07911424593, 0.04229505176,
                1.330207914, 1.454035996, 1.656271279, 1.841680039, 2.153642173,
                3.088973854, 3.108201582, 0.4882403422, 1.986610347, 1.918437741]
}

# Convert to DataFrame
df = pd.DataFrame(data)

# Calculate f_statis for verification
df['f_statis_calculated'] = df['Right Force'] + df['Left Force']

# Scatter plot
plt.scatter(df['fnav'], df['fstatis'])
plt.xlabel('fnav')
plt.ylabel('fstatis')
plt.title('Scatter plot of fnav vs. fstatis')
plt.grid(True)
plt.show()

# Calculate correlation coefficient
correlation_coefficient = np.corrcoef(df['fnav'], df['fstatis'])[0, 1]
print("Correlation coefficient:", correlation_coefficient)

# Linear Regression Analysis

# Independent variable (fstatis)
X = df['fstatis']
# Dependent variable (fnav)
y = df['fnav']

# Add a constant to the independent variable
X = sm.add_constant(X)

# Fit the regression model
model = sm.OLS(y, X).fit()

# Summary of the regression model
model_summary = model.summary()
print(model_summary)

# Residuals
residuals = model.resid

# Scatter plot with regression line
plt.scatter(df['fstatis'], df['fnav'], label='Data points')
plt.plot(df['fstatis'], model.fittedvalues, color='red', label='Regression line')
plt.xlabel('fstatis')
plt.ylabel('fnav')
plt.title('Scatter plot of fnav vs. fstatis with Regression Line')
plt.legend()
plt.grid(True)
plt.show()

# Residual plot
plt.scatter(model.fittedvalues, residuals)
plt.axhline(0, color='red', linestyle='--')
plt.xlabel('Fitted values')
plt.ylabel('Residuals')
plt.title('Residual plot')
plt.grid(True)
plt.show()
