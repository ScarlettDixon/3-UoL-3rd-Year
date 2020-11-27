import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from sklearn.cluster import KMeans

#data_train2 = pd.DataFrame.from_csv('kmeans.csv', header=None, index_col=None)
data_train =np.loadtxt(open("kmeans.csv", "rb"), delimiter=",")

#x_train = data_train.iloc[:,0].as_matrix()
#y_train = data_train.iloc[:,1].as_matrix()
x_train = data_train[:,0]
y_train = data_train[:,1]

k = KMeans(n_clusters=3).fit_predict(data_train)
plt.scatter(x_train, y_train, c=k)


plt.clf()
#plt.plot(x_train, y_train, 'bo')
plt.scatter(x_train, y_train, c=k)
plt.legend(('training points'))
plt.hold(True)
plt.show()

  