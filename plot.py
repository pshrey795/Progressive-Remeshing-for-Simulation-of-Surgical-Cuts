import matplotlib.pyplot as plt

error_values_0 = []
error_values_1 = []

with open("output0.txt","r") as f:
    for line in f:
        error_values_0.append(float(line))
        
with open("output1.txt","r") as f:
    for line in f:
        error_values_1.append(float(line))        
        
plt.plot(error_values_0, label = "Forward Euler")
plt.plot(error_values_1, label = "Backward Euler")
plt.xlabel('Time Steps')
plt.ylabel('Average Error')
plt.legend()
plt.show()
