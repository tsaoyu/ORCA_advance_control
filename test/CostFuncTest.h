#pragma once
#include <ct/optcon/optcon.h>

const size_t state_dim = 12;
const size_t control_dim = 4;

using namespace ct::core;
using namespace ct::optcon;

namespace orca{

namespace test
{

template <size_t state_dim, size_t control_dim>
void compareCostFunctionOutput(CostFunctionQuadratic<state_dim, control_dim>& costFunction,
    CostFunctionQuadratic<state_dim, control_dim>& costFunction2)
{
    ASSERT_NEAR(costFunction.evaluateIntermediate(), costFunction2.evaluateIntermediate(), 1e-9);
    ASSERT_NEAR(costFunction.evaluateTerminal(), costFunction2.evaluateTerminal(), 1e-9);

    ASSERT_TRUE(costFunction.stateDerivativeIntermediate().isApprox(costFunction2.stateDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.stateDerivativeTerminal().isApprox(costFunction2.stateDerivativeTerminal()));

    ASSERT_TRUE(
        costFunction.stateSecondDerivativeIntermediate().isApprox(costFunction2.stateSecondDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.stateSecondDerivativeTerminal().isApprox(costFunction2.stateSecondDerivativeTerminal()));

    ASSERT_TRUE(costFunction.controlDerivativeIntermediate().isApprox(costFunction2.controlDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.controlDerivativeTerminal().isApprox(costFunction2.controlDerivativeTerminal()));

    ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(
        costFunction2.controlSecondDerivativeIntermediate()));
    ASSERT_TRUE(
        costFunction.controlSecondDerivativeTerminal().isApprox(costFunction2.controlSecondDerivativeTerminal()));

    ASSERT_TRUE(
        costFunction.stateControlDerivativeIntermediate().isApprox(costFunction2.stateControlDerivativeIntermediate()));
    ASSERT_TRUE(costFunction.stateControlDerivativeTerminal().isApprox(costFunction2.stateControlDerivativeTerminal()));

    // second derivatives have to be symmetric
    ASSERT_TRUE(costFunction.stateSecondDerivativeIntermediate().isApprox(
        costFunction.stateSecondDerivativeIntermediate().transpose()));
    ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(
        costFunction.controlSecondDerivativeIntermediate().transpose()));
}

template <size_t state_dim, size_t control_dim>
void printCostFunctionOutput(CostFunctionQuadratic<state_dim, control_dim>& costFunction,
    CostFunctionQuadratic<state_dim, control_dim>& costFunction2)
{
    std::cout << "eval intermediate " << std::endl;
    std::cout << costFunction.evaluateIntermediate() << std::endl << std::endl;
    std::cout << costFunction2.evaluateIntermediate() << std::endl;

    std::cout << "eval terminal " << std::endl;
    std::cout << costFunction.evaluateTerminal() << std::endl << std::endl;
    std::cout << costFunction2.evaluateTerminal() << std::endl;

    std::cout << "eval stateDerivativeIntermediate " << std::endl;
    std::cout << costFunction.stateDerivativeIntermediate() << std::endl << std::endl;
    std::cout << costFunction2.stateDerivativeIntermediate() << std::endl;

    std::cout << "eval stateDerivativeIntermediate " << std::endl;
    std::cout << costFunction.stateDerivativeTerminal() << std::endl << std::endl;
    std::cout << costFunction2.stateDerivativeTerminal() << std::endl;

    std::cout << "eval stateSecondDerivativeIntermediate " << std::endl;
    std::cout << costFunction.stateSecondDerivativeIntermediate() << std::endl << std::endl;
    std::cout << costFunction2.stateSecondDerivativeIntermediate() << std::endl;

    std::cout << "eval stateSecondDerivativeTerminal " << std::endl;
    std::cout << costFunction.stateSecondDerivativeTerminal() << std::endl << std::endl;
    std::cout << costFunction2.stateSecondDerivativeTerminal() << std::endl;

    std::cout << "eval controlDerivativeIntermediate " << std::endl;
    std::cout << costFunction.controlDerivativeIntermediate() << std::endl << std::endl;
    std::cout << costFunction2.controlDerivativeIntermediate() << std::endl;

    std::cout << "eval controlDerivativeTerminal " << std::endl;
    std::cout << costFunction.controlDerivativeTerminal() << std::endl << std::endl;
    std::cout << costFunction2.controlDerivativeTerminal() << std::endl;

    std::cout << "eval controlSecondDerivativeIntermediate " << std::endl;
    std::cout << costFunction.controlSecondDerivativeIntermediate() << std::endl << std::endl;
    std::cout << costFunction2.controlSecondDerivativeIntermediate() << std::endl;

    std::cout << "eval controlSecondDerivativeIntermediate " << std::endl;
    std::cout << costFunction.controlSecondDerivativeTerminal() << std::endl << std::endl;
    std::cout << costFunction2.controlSecondDerivativeTerminal() << std::endl;

    std::cout << "eval stateControlDerivativeIntermediate " << std::endl;
    std::cout << costFunction.stateControlDerivativeIntermediate() << std::endl << std::endl;
    std::cout << costFunction2.stateControlDerivativeIntermediate() << std::endl;

    std::cout << "eval stateControlDerivativeTerminal " << std::endl;
    std::cout << costFunction.stateControlDerivativeTerminal() << std::endl << std::endl;
    std::cout << costFunction2.stateControlDerivativeTerminal() << std::endl;

    std::cout << "eval stateSecondDerivativeIntermediate " << std::endl;
    std::cout << costFunction.stateSecondDerivativeIntermediate() << std::endl << std::endl;
    std::cout << costFunction.stateSecondDerivativeIntermediate().transpose() << std::endl;

    std::cout << "eval controlSecondDerivativeIntermediate " << std::endl;
    std::cout << costFunction.controlSecondDerivativeIntermediate() << std::endl << std::endl;
    std::cout << costFunction.controlSecondDerivativeIntermediate().transpose() << std::endl;
}

TEST(CostFuncTest, ADQuadraticTest)
{
    const size_t nWeights = 2;
    const size_t nTests = 10;

    CostFunctionAnalytical<state_dim, control_dim> costFunction;
    CostFunctionAD<state_dim, control_dim> costFunctionAD;

    // intermediate cost terms
    std::shared_ptr<TermQuadratic<state_dim, control_dim, double>> termQuadratic_interm(
        new TermQuadratic<state_dim, control_dim>);
    std::shared_ptr<TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>> termQuadraticAD_interm(
        new TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>);

    // final cost terms
    std::shared_ptr<TermQuadratic<state_dim, control_dim, double>> termQuadratic_final(
        new TermQuadratic<state_dim, control_dim>);
    std::shared_ptr<TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>> termQuadraticAD_final(
        new TermQuadratic<state_dim, control_dim, double, ct::core::ADCGScalar>);

    costFunction.addIntermediateTerm(termQuadratic_interm, true);
    costFunctionAD.addIntermediateADTerm(termQuadraticAD_interm, true);
    costFunction.addFinalTerm(termQuadratic_final, true);
    costFunctionAD.addFinalADTerm(termQuadraticAD_final, true);

    Eigen::Matrix<double, state_dim, state_dim> Q_interm;
    Eigen::Matrix<double, state_dim, state_dim> Q_final;
    Eigen::Matrix<double, control_dim, control_dim> R;

    ct::core::StateVector<state_dim> x_ref;
    ct::core::ControlVector<control_dim> u_ref;

    for (size_t i = 0; i < nWeights; i++)
    {
        try
        {
            Q_interm.setRandom();
            Q_final.setRandom();
            R.setRandom();
            x_ref.setRandom();
            u_ref.setRandom();

            if (i == 0)
            {
                Q_interm.setZero();
                Q_final.setZero();
                R.setZero();
                x_ref.setZero();
                u_ref.setZero();
            }

            if (i == 1)
            {
                Q_interm.setIdentity();
                Q_final.setIdentity();
                R.setZero();
                x_ref.setConstant(10.0);
                u_ref.setConstant(10.0);
            }


            Q_interm += Q_interm.transpose().eval();  // make symmetric
            R += R.transpose().eval();                // make symmetric
            Q_final += Q_final.transpose().eval();    // make symmetric

            termQuadratic_interm->setWeights(Q_interm, R);
            termQuadraticAD_interm->setWeights(Q_interm, R);
            termQuadratic_interm->setStateAndControlReference(x_ref, u_ref);
            termQuadraticAD_interm->setStateAndControlReference(x_ref, u_ref);

            termQuadratic_final->setWeights(Q_final, R);
            termQuadraticAD_final->setWeights(Q_final, R);
            termQuadratic_final->setStateAndControlReference(x_ref, u_ref);
            termQuadraticAD_final->setStateAndControlReference(x_ref, u_ref);

            costFunctionAD.initialize();

            // create cloned cost function
            std::shared_ptr<CostFunctionAD<state_dim, control_dim>> costFunctionAD_clone(costFunctionAD.clone());

            for (size_t j = 0; j < nTests; j++)
            {
                ct::core::StateVector<state_dim> x;
                ct::core::ControlVector<control_dim> u;
                x.setRandom();
                u.setRandom();

                if (j == 0)
                {
                    x.setZero();
                    u.setZero();
                }

                costFunction.setCurrentStateAndControl(x, u, 1.0);
                costFunctionAD.setCurrentStateAndControl(x, u, 1.0);
                costFunctionAD_clone->setCurrentStateAndControl(x, u, 1.0);

                printCostFunctionOutput(costFunction, costFunctionAD);
                compareCostFunctionOutput(costFunction, costFunctionAD);
                compareCostFunctionOutput(costFunction, *costFunctionAD_clone);

                // now some manual assertions
                ASSERT_TRUE(costFunction.stateDerivativeIntermediate().isApprox(2 * Q_interm * (x - x_ref)));
                ASSERT_TRUE(costFunction.stateDerivativeTerminal().isApprox(2 * Q_final * (x - x_ref)));

                ASSERT_TRUE(costFunction.stateSecondDerivativeIntermediate().isApprox(2 * Q_interm));
                ASSERT_TRUE(costFunction.stateSecondDerivativeTerminal().isApprox(2 * Q_final));

                ASSERT_TRUE(costFunction.controlDerivativeIntermediate().isApprox(2 * R * (u - u_ref)));

                ASSERT_TRUE(costFunction.controlSecondDerivativeIntermediate().isApprox(2 * R));
            }
        } catch (std::exception& e)
        {
            FAIL();
        }
    }
}

TEST(CostFuncTest, ADQuadMultTest)
{
    const size_t nWeights = 3;
    const size_t nTests = 10;

    CostFunctionAnalytical<state_dim, control_dim> costFunction;
    CostFunctionAD<state_dim, control_dim> costFunctionAD;

    std::shared_ptr<TermQuadMult<state_dim, control_dim, double>> termQuadMult(
        new TermQuadMult<state_dim, control_dim>);
    std::shared_ptr<TermQuadMult<state_dim, control_dim, double, ct::core::ADCGScalar>> termQuadMultAD(
        new TermQuadMult<state_dim, control_dim, double, ct::core::ADCGScalar>);

    std::shared_ptr<TermMixed<state_dim, control_dim, double>> termMixed(new TermMixed<state_dim, control_dim, double>);
    std::shared_ptr<TermMixed<state_dim, control_dim, double, ct::core::ADCGScalar>> termMixedAD(
        new TermMixed<state_dim, control_dim, double, ct::core::ADCGScalar>);

    costFunction.addIntermediateTerm(termQuadMult);
    costFunctionAD.addIntermediateADTerm(termQuadMultAD);
    costFunctionAD.initialize();

    Eigen::Matrix<double, state_dim, state_dim> Q;
    Eigen::Matrix<double, control_dim, control_dim> R;
    Eigen::Matrix<double, control_dim, state_dim> P;

    ct::core::StateVector<state_dim> x_ref;
    ct::core::ControlVector<control_dim> u_ref;

    for (size_t i = 0; i < nWeights; i++)
    {
        try
        {
            Q.setRandom();
            R.setRandom();
            P.setRandom();
            x_ref.setRandom();
            u_ref.setRandom();

            if (i == 0)
            {
                Q.setZero();
                R.setZero();
                P.setZero();
                x_ref.setZero();
                u_ref.setZero();
            }

            if (i == 1)
            {
                Q.setIdentity();
                R.setIdentity();
                P.setZero();
                x_ref.setConstant(10.0);
                u_ref.setConstant(10.0);
            }

            Q += Q.transpose().eval();  // make symmetric
            R += R.transpose().eval();  // make symmetric

            termQuadMult->setWeights(Q, R);
            termQuadMultAD->setWeights(Q, R);
            termQuadMult->setStateAndControlReference(x_ref, u_ref);
            termQuadMultAD->setStateAndControlReference(x_ref, u_ref);

            costFunctionAD.initialize();

            // create cloned cost function
            std::shared_ptr<CostFunctionAD<state_dim, control_dim>> costFunctionAD_clone(costFunctionAD.clone());

            for (size_t j = 0; j < nTests; j++)
            {
                ct::core::StateVector<state_dim> x;
                ct::core::ControlVector<control_dim> u;
                x.setRandom();
                u.setRandom();
                double t = 3.14;

                if (j == 0)
                {
                    x.setZero();
                    u.setZero();
                }

                costFunction.setCurrentStateAndControl(x, u, t);
                costFunctionAD.setCurrentStateAndControl(x, u, t);
                costFunctionAD_clone->setCurrentStateAndControl(x, u, t);

                compareCostFunctionOutput(costFunction, costFunctionAD);
                compareCostFunctionOutput(*costFunctionAD_clone, costFunctionAD);
            }
        } catch (std::exception& e)
        {
            FAIL();
        }
    }
}


TEST(CostFuncTest, TrackingTermTest)
{
    const size_t state_dim = 12;
    const size_t control_dim = 4;

    // analytical costfunction
    std::shared_ptr<CostFunctionAnalytical<state_dim, control_dim>> costFunction(
        new CostFunctionAnalytical<state_dim, control_dim>());

    Eigen::Matrix<double, state_dim, state_dim> Q;
    Eigen::Matrix<double, control_dim, control_dim> R;
    Q.setIdentity();
    R.setIdentity();

    // create a reference trajectory and fill it with random values
    ct::core::StateTrajectory<state_dim> stateTraj;
    ct::core::ControlTrajectory<control_dim> controlTraj;
    size_t trajSize = 50;
    bool timeIsAbsolute = true;
    for (size_t i = 0; i < trajSize; ++i)
    {
        stateTraj.push_back(ct::core::StateVector<state_dim>::Random(), double(i), timeIsAbsolute);
        controlTraj.push_back(ct::core::ControlVector<control_dim>::Random(), double(i), timeIsAbsolute);
    }

    std::shared_ptr<TermQuadTracking<state_dim, control_dim>> trackingTerm(new TermQuadTracking<state_dim, control_dim>(
        Q, R, ct::core::InterpolationType::LIN, ct::core::InterpolationType::ZOH, true));

    trackingTerm->setStateAndControlReference(stateTraj, controlTraj);
    costFunction->addIntermediateTerm(trackingTerm);

    ct::core::StateVector<state_dim> x;
    ct::core::ControlVector<control_dim> u;
    x.setRandom();
    u.setRandom();
    double t = 0.0;

    costFunction->setCurrentStateAndControl(x, u, t);

    ASSERT_TRUE(costFunction->stateDerivativeIntermediateTest());
    ASSERT_TRUE(costFunction->controlDerivativeIntermediateTest());
}

TEST(CostFuncTest, TermSmoothAbsTest)
{
    const size_t state_dim = 12;
    const size_t control_dim = 4;

    // analytical costfunction
    std::shared_ptr<CostFunctionAnalytical<state_dim, control_dim>> costFunction(
        new CostFunctionAnalytical<state_dim, control_dim>());

    // autodiff costfunction
    using CGScalar = typename CostFunctionAD<state_dim, control_dim>::CGScalar;
    std::shared_ptr<CostFunctionAD<state_dim, control_dim>> costFunctionAD(
        new CostFunctionAD<state_dim, control_dim>());

    Eigen::Matrix<double, state_dim, 1> a, x_ref;
    a.setRandom();
    x_ref.setRandom();
    Eigen::Matrix<double, control_dim, 1> b, u_ref;
    b.setRandom();
    u_ref.setRandom();
    double alpha = 0.5;

    std::shared_ptr<TermSmoothAbs<state_dim, control_dim>> smoothAbsTerm(
        new TermSmoothAbs<state_dim, control_dim>(a, x_ref, b, u_ref, alpha));

    std::shared_ptr<TermSmoothAbs<state_dim, control_dim, double, CGScalar>> smoothAbsTermAD(
        new TermSmoothAbs<state_dim, control_dim, double, CGScalar>(a, x_ref, b, u_ref, alpha));

    costFunction->addIntermediateTerm(smoothAbsTerm);
    costFunctionAD->addIntermediateADTerm(smoothAbsTermAD);
    costFunctionAD->initialize();

    ct::core::StateVector<state_dim> x;
    ct::core::ControlVector<control_dim> u;
    x.setRandom();
    u.setRandom();
    double t = 0.0;

    costFunction->setCurrentStateAndControl(x, u, t);
    ASSERT_TRUE(costFunction->stateDerivativeIntermediateTest());
    ASSERT_TRUE(costFunction->controlDerivativeIntermediateTest());

    for (int i = 0; i < 100; i++)
    {
        x.setRandom();
        u.setRandom();

        costFunction->setCurrentStateAndControl(x, u, t);
        costFunctionAD->setCurrentStateAndControl(x, u, t);

        compareCostFunctionOutput(*costFunction, *costFunctionAD);
    }
}



} // End of namespace test


} // End of namespace orca