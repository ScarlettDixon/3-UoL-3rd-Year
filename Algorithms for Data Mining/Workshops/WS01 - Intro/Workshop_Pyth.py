import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

def input(train1, test1):
	data_train = pd.DataFrame.from_csv(train1) #Import from 
	data_test = pd.DataFrame.from_csv(test1)
	return data_train, data_test
	
def trainxy (d_train, d_test):
	x_train = d_train['x'].as_matrix()
	y_train = d_train['y'].as_matrix()

	x_test = d_test['x'].as_matrix()
	y_test = d_test['y'].as_matrix()
	return x_train, y_train, x_test, y_test
	
def plot_simp(x_train,y_train,x_test,y_test):
	plt.clf()
	plt.plot(x_train,y_train, 'bo')
	plt.plot(x_test,y_test, 'g')
	plt.legend(('training points', 'ground truth'))
	plt.hold(True)
	plt.savefig('trainingdata.png')
	plt.show()
	
def calc_x_tilda (x_train):	
	x_tild = np.column_stack((np.ones(x_train.shape), x_train))
	return x_tild
	
def cal_derive (fx):
    print("Calulate the derivative")
    print("Constants: ∂a / ∂x = 0")
    print("Linear term: ∂Ax / ∂x = A")
    print("Quadratic terms: ")
    print("∂xTx / ∂x = 2xT or ∂xTAx / ∂x= 2x TA")
    print("Can use the chain rule if symmetic matrix")
	
(d_train, d_test) = input('regression_train.csv', 'regression_test.csv')
(x_train, y_train, x_test, y_test) = trainxy (d_train, d_test)
plot_simp(x_train,y_train,x_test,y_test)
(x_tild) = calc_x_tilda (x_train)