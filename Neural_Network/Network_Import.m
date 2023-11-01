

%Specify keras network file name
modelfile = 'new.h5';

%convert keras network to mat 
% net = importNetworkFromTensorFlow(modelfile)
net = importKerasNetwork(modelfile)

%save converted file
save  net

%show the network
plot(net)