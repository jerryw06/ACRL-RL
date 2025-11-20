# PPO Implementation Guide

## Overview

I've implemented **Proximal Policy Optimization (PPO)** for your PX4 drone control task. PPO is the gold standard for continuous control and offers significant advantages over vanilla REINFORCE.

## What Changed

### 1. **Added Value Network (Critic)**
- **File**: `include/policy_network.hpp`, `src/policy_network.cpp`
- **Architecture**: 4 layers × 512 hidden units
- **Purpose**: Estimates state values V(s) for variance reduction

### 2. **Implemented GAE (Generalized Advantage Estimation)**
- **Function**: `compute_gae()` in `train_rl.cpp`
- **Formula**: 
  - TD error: δ_t = r_t + γV(s_{t+1}) - V(s_t)
  - Advantage: A_t = δ_t + (γλ)A_{t+1}
  - Return: R_t = A_t + V(s_t)
- **Benefits**: Balances bias-variance tradeoff (λ = 0.95)

### 3. **PPO Clipped Objective**
- **Formula**: L^CLIP = min(r_t A_t, clip(r_t, 1-ε, 1+ε) A_t)
  - r_t = exp(log π_new - log π_old) is the probability ratio
  - ε = 0.2 is the clipping parameter
- **Purpose**: Prevents destructively large policy updates

### 4. **Multi-Epoch Updates**
- **Epochs per episode**: 10
- **Mini-batch size**: 64
- **Why**: Reuses collected data efficiently (better sample efficiency)

### 5. **Entropy Bonus**
- **Coefficient**: 0.01
- **Formula**: H = E[log(σ√(2πe))] for Gaussian policy
- **Purpose**: Encourages exploration during training

## PPO Hyperparameters

```cpp
struct PPOConfig {
    double gamma = 0.99;           // Discount factor
    double gae_lambda = 0.95;      // GAE lambda
    double clip_epsilon = 0.2;     // PPO clipping
    int update_epochs = 10;        // Update iterations per episode
    int mini_batch_size = 64;      // Mini-batch size
    double value_loss_coef = 0.5;  // Value loss weight
    double entropy_coef = 0.01;    // Entropy bonus weight
    double max_grad_norm = 0.5;    // Gradient clipping
    double learning_rate = 3e-4;   // Learning rate
};
```

## Key Improvements Over REINFORCE

| Feature | REINFORCE | PPO |
|---------|-----------|-----|
| Sample Efficiency | Low (1 update per episode) | High (10 epochs × mini-batches) |
| Variance | High (Monte Carlo returns) | Low (GAE with value baseline) |
| Stability | Poor (large policy changes) | Excellent (clipped updates) |
| Exploration | Random sampling only | Entropy bonus + sampling |
| Convergence Speed | Slow | Fast |

## Training Output

You'll now see additional metrics:
```
Episode 1 | steps=200 | return=15.234 | π_loss=0.0123 | v_loss=2.456 | entropy=1.234
```

- **π_loss**: Policy loss (lower = better policy gradient alignment)
- **v_loss**: Value loss (lower = better value predictions)
- **entropy**: Policy entropy (higher = more exploration)

## Saved Artifacts

- `best_policy.pt` - Best actor network
- `best_value.pt` - Best critic network
- `final_policy.pt` - Final actor network
- `final_value.pt` - Final critic network
- `policy_config.json` - Full PPO configuration including hyperparameters

## Running PPO Training

Same command as before:
```bash
cd ~/PegasusSimulator_51/examples/RL_test/ACRL-RL/RL_with_cpp
./run_training_cyclone.sh
```

## Expected Behavior

1. **Faster Learning**: Should reach good policies in ~50-100 episodes vs 150+ for REINFORCE
2. **More Stable**: Smoother learning curves with less variance
3. **Better Final Performance**: Higher average returns due to better sample efficiency

## Tuning Tips

If training is unstable or slow:

1. **Reduce clip_epsilon** (0.2 → 0.1) for more conservative updates
2. **Increase mini_batch_size** (64 → 128) for more stable gradients (if episodes are long)
3. **Adjust value_loss_coef** (0.5 → 1.0) if value predictions are poor
4. **Decrease learning_rate** (3e-4 → 1e-4) if loss oscillates

If exploration is insufficient:

1. **Increase entropy_coef** (0.01 → 0.02)
2. **Slow down log_std decay** (modify initial log_std in policy_network.cpp)

## Algorithm Flow

```
For each episode:
  1. Collect trajectory: {s_t, a_t, r_t, log π(a_t|s_t), V(s_t)}
  2. Compute GAE advantages and returns
  3. For 10 epochs:
     4. Shuffle data into mini-batches
     5. For each mini-batch:
        6. Compute new log π(a|s) and V(s)
        7. Compute PPO clipped loss
        8. Compute value MSE loss
        9. Compute entropy bonus
        10. Backprop: L_total = L_clip + 0.5*L_value - 0.01*H
        11. Clip gradients and update networks
```

## Why PPO for Drone Control?

1. **Continuous Actions**: Gaussian policy with PPO handles smooth acceleration commands
2. **Safety**: Clipped objective prevents catastrophic policy updates
3. **Sample Efficiency**: Critical for real robot/sim training (fewer episodes needed)
4. **Industry Standard**: Used in OpenAI Five, Google DeepMind robotics, autonomous vehicles

## Comparison with stable-baselines3

This implementation matches stable-baselines3 PPO with:
- ✅ GAE advantage estimation
- ✅ Clipped surrogate objective
- ✅ Value function baseline
- ✅ Entropy bonus
- ✅ Multiple update epochs
- ✅ Mini-batch updates
- ✅ Gradient clipping

Key difference: We use continuous OFFBOARD heartbeat (your fix) which stable-baselines3 doesn't handle.

## Next Steps

1. **Run training** - Should see faster convergence
2. **Monitor metrics** - Check if π_loss, v_loss stabilize after ~50 episodes
3. **Tune if needed** - Adjust hyperparameters based on learning curves
4. **Evaluate** - Compare final return distribution vs old REINFORCE

## References

- [PPO Paper](https://arxiv.org/abs/1707.06347) - Schulman et al. 2017
- [GAE Paper](https://arxiv.org/abs/1506.02438) - Schulman et al. 2016
- [Implementation Details](https://iclr-blog-track.github.io/2022/03/25/ppo-implementation-details/) - What matters in PPO

---

**Status**: ✅ Built successfully in 38.7s  
**Ready to train**: Yes, run `./run_training_cyclone.sh`
