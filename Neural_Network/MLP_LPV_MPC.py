#!/usr/bin/env python
# coding: utf-8

# In[1]:




# Import all needed libraries and sublibraries

import tensorflow.keras
import tensorflow as tf
from tensorflow.keras import models, regularizers
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Activation, BatchNormalization,Dropout, LeakyReLU
from tensorflow.keras.callbacks import ModelCheckpoint,EarlyStopping
from tensorflow.keras.optimizers import Adam, SGD, RMSprop, Adagrad, Adadelta
from tensorflow.keras.metrics import mean_absolute_error,mean_squared_error
# from tensorflow.keras.utils import np_utils
from sklearn.preprocessing import StandardScaler,MinMaxScaler,RobustScaler,MaxAbsScaler
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
get_ipython().run_line_magic('matplotlib', 'inline')



# In[2]:


# Import input (x) and output (y) data, and asign these to df1 and df1
# df = pd.read_csv('data.csv')
df = pd.read_excel('datax.xlsx')
df1 = df.iloc[:,2:7 ]
df2 = df.iloc[:,0:2 ]


# In[3]:




# Scale both training and testing input data
scalerx = StandardScaler() 
scalery = StandardScaler() 

# scaler = StandardScaler() 
# scaler = MinMaxScaler() 
# scaler = RobustScaler() 
# scaler = MaxAbsScaler() 

# df2 = scaler.fit_transform(df2)

Df1 = scalerx.fit_transform(df1)

Df2 = scalery.fit_transform(df2)


# scaler.mean_
# scaler.scale_


# In[4]:


scalerx.mean_


# In[5]:


scalerx.scale_


# In[6]:


scalery.mean_


# In[7]:


scalery.scale_


# In[8]:




# Split the data into input (x) training and testing data, and ouput (y) training and testing data, 
# with training data being 80% of the data, and testing data being the remaining 20% of the data

X_train, X_test, y_train, y_test = train_test_split(Df1 , Df2, test_size=0.2)


# In[9]:


sns.displot(X_train[:,4])
# sns.histplot(X_test.iloc[:,0])


# In[10]:




# # Scale both training and testing input data
# scaler = StandardScaler() 

# # scaler = MinMaxScaler() 
# # scaler = RobustScaler() 
# # df2 = scaler.fit_transform(df2)

# X_train = scaler.fit_transform(X_train)

# X_test = scaler.fit_transform(X_test)

# y_train = scaler.fit_transform(y_train)

# y_test = scaler.fit_transform(y_test)

# scalerx.mean_
# scalery.scale_


# In[11]:


# sns.displot(X_train[:,4])
sns.histplot(X_test[:,0])


# In[12]:



model = Sequential([Dense(16, input_shape=(5,), activation='sigmoid'),
                    Dense(28, activation='sigmoid'),
                    Dense(14, activation='sigmoid'),
                    Dense(9, activation='sigmoid'),
                    Dense(2),
                   ])

# # compile model
model.compile(Adam(lr=0.0015),loss='mse')

# model = models.load_model('corneringg.h5')


#create callback
path = 'new.h5'
#monitor = EarlyStopping(monitor='val_loss',min_delta=1e-3,patience=20,verbose=1,restore_best_weights=True)
checkpoint = ModelCheckpoint(path, monitor="val_loss", verbose=1, save_best_only=True, mode='min')
callbacks_list = [checkpoint]

#model summary
model.summary()




# In[13]:


#Fits model
history = model.fit(X_train, y_train, epochs = 2500, validation_split = 0.1,callbacks = callbacks_list,                    batch_size =64, shuffle = True, verbose = 1)

history_dict=history.history


# In[15]:


history_dict=history.history
#Plots model's training cost/loss and model's validation split cost/loss
loss_values = history_dict['loss']
val_loss_values=history_dict['val_loss']
# plt.figure()

fig, ax = plt.subplots()

plt.plot(loss_values,'b',label='training loss')
plt.plot(val_loss_values,'r',label='validation loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(loc='upper right')
plt.show()


fig.savefig('loss.png', format='png', dpi=1200)


# In[16]:




# Runs model (the one with the activation function, although this doesn't really matter as they perform the same) 
# with its current weights on the training and testing data
y_train_pred = model.predict(X_train)
y_test_pred = model.predict(X_test)

# Calculates and prints r2 score of training and testing data
print("The R2 score on the Train set is:\t{:0.3f}".format(r2_score(y_train, y_train_pred)))
print("The R2 score on the Test set is:\t{:0.3f}".format(r2_score(y_test, y_test_pred)))


# In[17]:


y_test = scalery.inverse_transform(y_test)
y_test_pred = scalery.inverse_transform(y_test_pred)


# In[18]:


#plot predictions
fig, ax = plt.subplots()

plt.title('Predictions VS Ground_truth')
plt.scatter(y_test[:,0],y_test_pred[:,0])
plt.ylabel('Predictions')
plt.xlabel('True value')
# plt.legend(['Ground_truth', 'Predictions'], loc='upper left')
plt.show()
fig.savefig('predVStruth.png', format='png', dpi=1200)


# In[19]:


def chart_regression(pred,y,sort=True):
    #t = pd.DataFrame({'preds':preds,'y':y.flatten()})
    #if sort:
        #t.sort.values(by = ['y'], inplace = True)
    fig, ax = plt.subplots()
    a = plt.plot(y.tolist(),label='expected')
    b = plt.plot(pred.tolist(),label='prediction')
    plt.ylabel('output')
    plt.xlabel('datapoint')
    plt.legend(loc='upper right')
    plt.show()
    fig.savefig('pred_expect.png', format='png', dpi=1200)


# In[23]:


chart_regression(y_test_pred[0:70,0],y_test[0:70,0],sort=True)


# In[17]:


y_test_pred = scaler.inverse_transform(y_test_pred)
y_test_pred


# In[ ]:


# y_preds = model.predict(X_test)
#plot predictions
plt.title('Predictions VS Ground_truth')
plt.legend(['Ground_truth', 'Predictions'], loc='upper right')
plt.scatter(y_test,y_test_pred)
plt.ylabel('Value')
plt.xlabel('samples')
plt.show()


# In[243]:


def chart_regression(pred,y,sort=True):
    #t = pd.DataFrame({'preds':preds,'y':y.flatten()})
    #if sort:
        #t.sort.values(by = ['y'], inplace = True)
    a = plt.plot(y.tolist(),label='expected')
    b = plt.plot(pred.tolist(),label='prediction')
    plt.ylabel('output')
    plt.legend()
    plt.show()


# In[244]:


chart_regression(y_test_pred[0:70,1],y_test[0:70,1],sort=True)


# In[123]:


y_test_pred[0:70,0]


# In[ ]:




