#include "prx/external/svm/svm.h"
#include "prx/utilities/defs.hpp"

using namespace prx;

#define Malloc(type,n) (type *)malloc((n)*sizeof(type))

int main(int argc, char* argv[])
{
    try
    {
        init_random(210896);

        svm_parameter param;
        param.svm_type = C_SVC;
        param.kernel_type = RBF;
        param.gamma = 0.2;
        param.cache_size = 200;
        param.eps = 1e-3;
        param.C = 1;
        param.nr_weight = 0;
        param.weight_label = NULL;
        param.weight = NULL;
        param.shrinking = 1;
        param.probability = 0;

        svm_problem prob;
        prob.l = 4;
        prob.y = Malloc(double,prob.l);
        prob.x = Malloc(struct svm_node *,prob.l);

        struct svm_model *model;
        struct svm_node *x_space;

        x_space = Malloc(struct svm_node, (2+1) * prob.l);

        prob.y[0] = prob.y[3] = 0;
        prob.y[1] = prob.y[2] = 1;

        std::vector<std::vector<double>> data = {
            {0,0},
            {0,1},
            {1,0},
            {1,1}
        };

        int j = 0;
        for (int i=0;i < prob.l; ++i)
        {
            prob.x[i] = &x_space[j];
            for (int k=0; k<data[i].size(); ++k, ++j)
            {
                x_space[j].index=k+1; 
                x_space[j].value=data[i][k]; 
            }
            x_space[j].index=-1;
            x_space[j].value=0;
            j++;

        }

        model = svm_train(&prob, &param);

        // Display the predictions.
        for (int i = 0; i < prob.l; ++i)
        {
            double prediction = svm_predict(model, prob.x[i]);
            std::cout << "Prediction: " << prediction << std::endl;
        }

    }
    catch(const prx_assert_t& e)
    {
        std::cout << e.get_message() << '\n';
    }
    
}