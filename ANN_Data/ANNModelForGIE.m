%% 1. Load CSV files
% Adjust filenames/path if necessary.
trainData = readmatrix('C:\Users\ACER\OneDrive - Umich\My PhD\Research\Topic discussion\MLforLargescaleIBRps\PSEstimation\ANN_Based_GI_estimation\ECCE_conf\Data\X_trainTestt.csv');
devData   = readmatrix('C:\Users\ACER\OneDrive - Umich\My PhD\Research\Topic discussion\MLforLargescaleIBRps\PSEstimation\ANN_Based_GI_estimation\ECCE_conf\Data\X_dev.csv');
testData  = readmatrix('C:\Users\ACER\OneDrive - Umich\My PhD\Research\Topic discussion\MLforLargescaleIBRps\PSEstimation\ANN_Based_GI_estimation\ECCE_conf\Data\X_test.csv');

%% 2. Split into features and targets
% Assumes the last two columns are the targets: [Lg, Rg].
X_train = trainData(:, 1:end-2);
y_train = trainData(:, end-1:end);   % Two-column target: [Lg, Rg]

X_dev   = devData(:, 1:end-2);
y_dev   = devData(:, end-1:end);

X_test  = testData(:, 1:end-2);
y_test  = testData(:, end-1:end);

%% 3. Scale the data using training set statistics
mu    = mean(X_train);
sigma = std(X_train);

X_train_scaled = (X_train - mu) ./ sigma;
X_dev_scaled   = (X_dev - mu) ./ sigma;
X_test_scaled  = (X_test - mu) ./ sigma;

%% 4. Train an ANN Model with Two Outputs
% Define the network architecture, e.g., one hidden layer with 10 neurons.
hiddenLayerSize = 8;
net = feedforwardnet(hiddenLayerSize);
net.trainFcn    = 'trainlm';  % Levenberg-Marquardt algorithm
net.performFcn  = 'mse';      % Mean Squared Error
% overfitting
net.performParam.regularization = 0.1;
% Train the network.
% Note: The network expects the input and target data as columns.
net = train(net, X_train_scaled', y_train');

% 
%% 5. Evaluate the ANN Model on the Development Set
y_dev_pred = net(X_dev_scaled');  % Predictions will be 2 x N
y_dev_pred = y_dev_pred';         % Convert to N x 2

% Compute Mean Squared Error for each target
dev_mse_Lg = mean((y_dev(:,1) - y_dev_pred(:,1)).^2);
dev_mse_Rg = mean((y_dev(:,2) - y_dev_pred(:,2)).^2);

fprintf('ANN Development Set MSE for Lg: %.4f\n', dev_mse_Lg);
fprintf('ANN Development Set MSE for Rg: %.4f\n', dev_mse_Rg);

% Optionally, compute R² for each target
dev_r2_Lg = 1 - sum((y_dev(:,1) - y_dev_pred(:,1)).^2) / sum((y_dev(:,1) - mean(y_dev(:,1))).^2);
dev_r2_Rg = 1 - sum((y_dev(:,2) - y_dev_pred(:,2)).^2) / sum((y_dev(:,2) - mean(y_dev(:,2))).^2);

fprintf('ANN Development Set R² for Lg: %.4f\n', dev_r2_Lg);
fprintf('ANN Development Set R² for Rg: %.4f\n', dev_r2_Rg);

%% 6. Evaluate the ANN Model on the Testing Set
y_test_pred = net(X_test_scaled');
y_test_pred = y_test_pred';

test_mse_Lg = mean((y_test(:,1) - y_test_pred(:,1)).^2);
test_mse_Rg = mean((y_test(:,2) - y_test_pred(:,2)).^2);

fprintf('ANN Testing Set MSE for Lg: %.4f\n', test_mse_Lg);
fprintf('ANN Testing Set MSE for Rg: %.4f\n', test_mse_Rg);

test_r2_Lg = 1 - sum((y_test(:,1) - y_test_pred(:,1)).^2) / sum((y_test(:,1) - mean(y_test(:,1))).^2);
test_r2_Rg = 1 - sum((y_test(:,2) - y_test_pred(:,2)).^2) / sum((y_test(:,2) - mean(y_test(:,2))).^2);

fprintf('ANN Testing Set R² for Lg: %.4f\n', test_r2_Lg);
fprintf('ANN Testing Set R² for Rg: %.4f\n', test_r2_Rg);

%% 7. Visualization: Predictions vs. Actual (Testing Set)
% Plot for Lg
figure;
scatter(y_test(:,1), y_test_pred(:,1), 'filled');
hold on;
plot([min(y_test(:,1)) max(y_test(:,1))], [min(y_test(:,1)) max(y_test(:,1))], 'r--', 'LineWidth', 2);
xlabel('Actual Lg');
ylabel('Predicted Lg');
title('ANN Predictions vs Actual for Lg');
legend('Predictions', 'Ideal Fit', 'Location', 'best');
grid on;
% 
% 
% Plot for Rg
figure;
scatter(y_test(:,2), y_test_pred(:,2), 'filled');
hold on;
plot([min(y_test(:,2)) max(y_test(:,2))], [min(y_test(:,2)) max(y_test(:,2))], 'r--', 'LineWidth', 2);
xlabel('Actual Rg');
ylabel('Predicted Rg');
title('ANN Predictions vs Actual for Rg');
legend('Predictions', 'Ideal Fit', 'Location', 'best');
grid on;


% %% 10. Save the ANN Model and Scaling Parameters
save('ann_model_params.mat', 'net', 'mu', 'sigma');
% %% 11. Prepare the ANN for Simulink Simulation
gensim(net);