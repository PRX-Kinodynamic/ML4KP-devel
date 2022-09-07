#pragma once
#include "prx/utilities/defs.hpp"
#include "prx/utilities/learned_modules/learned_modules_utils.hpp"

#include "prx/external/svm/svm.h"

class termination_classifier_t
{
    private:
    struct svm_model* model;
    struct svm_node* x_space_train;
    struct svm_node* x_space_test;
    svm_parameter param;
    svm_problem prob_train, prob_test;

    protected:
    bool normalize_input;
    std::vector<double> upper_bounds, lower_bounds;
    double accuracy;

    public:
    termination_classifier_t(param_loader params)
    {
        init(params);
    }

    termination_classifier_t() {}

    ~termination_classifier_t()
    {
        svm_destroy_param(&param);
        svm_free_and_destroy_model(&model);
        free(prob_train.y);
        free(prob_train.x);
        free(prob_test.y);
        free(prob_test.x);
        free(x_space_train);
        free(x_space_test);
    }

    double get_accuracy()
    {
        return accuracy;
    }

    void init(param_loader params)
    {
        lower_bounds = params["/plant/state_space_lower_bound"].as<std::vector<double>>();
        upper_bounds = params["/plant/state_space_upper_bound"].as<std::vector<double>>();

        normalize_input = params["/termination_classifier/normalize_input"].as<bool>();

        param.svm_type = C_SVC;
        param.kernel_type = RBF;

        param.gamma = params["/termination_classifier/gamma"].as<double>();
        param.C = params["/termination_classifier/C"].as<double>();

        param.cache_size = 200;
        param.eps = 1e-3;
        param.nr_weight = 0;
        param.weight_label = NULL;
        param.weight = NULL;
        param.shrinking = 0;
        param.probability = 0;
    }

    void train(std::vector<std::vector<double>> data, std::vector<double> labels)
    {
        accuracy = 0.0;
        prob_train.l = data.size();
        prob_train.y = Malloc(double,prob_train.l);
        prob_train.x = Malloc(struct svm_node *,prob_train.l);

        prob_test.l = 1;
        prob_test.y = Malloc(double,1);
        prob_test.x = Malloc(struct svm_node *,1);

        x_space_train = Malloc(struct svm_node, (data[0].size()+1) * prob_train.l);
        x_space_test  = Malloc(struct svm_node, (data[0].size()+1));

        int j = 0;
        for (int i = 0; i < prob_train.l; ++i)
        {
            prob_train.x[i] = &x_space_train[j];
            prob_train.y[i] = labels[i];
            if (normalize_input)
            {
                data[i] = normalize_vector(data[i], lower_bounds, upper_bounds);
            }
            for (int k = 0; k < data[i].size(); ++k, ++j)
            {
                x_space_train[j].index = k+1; 
                x_space_train[j].value = data[i][k]; 
            }
            x_space_train[j].index = -1;
            x_space_train[j].value = 0;
            j++;
        }

        model = svm_train(&prob_train, &param);

        for (int i = 0; i < prob_train.l; ++i)
        {
            double prediction = svm_predict(model, prob_train.x[i]);
            if (prediction == prob_train.y[i])
                accuracy++;
        }

        accuracy /= prob_train.l;

        std::cout << "Training accuracy: " << accuracy << std::endl;
    }

    bool predict (std::vector<double> data)
    {
        if (normalize_input)
        {
            data = normalize_vector(data, lower_bounds, upper_bounds);
        }
        
        prob_test.x[0] = &x_space_test[0];
        for (int k = 0; k < data.size(); ++k)
        {
            x_space_test[k].index = k+1; 
            x_space_test[k].value = data[k]; 
        }
        x_space_test[data.size()].index = -1;
        x_space_test[data.size()].value = 0;

        double prediction = svm_predict(model, prob_test.x[0]);

        return prediction;
    }
};