# -*- coding: utf-8 -*-
"""Agent_utils.ipynb

Automatically generated by Colab.

Original file is located at
    https://colab.research.google.com/github/Gaianeve/FUFONE/blob/main/PPO/Agent_utils.ipynb

# Agent related functions.
"""

#libraries
import gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import time
from wrappers import HistoryWrapper
from environment import vectorize_env

"""## Annealing
Takes care of annealing, AKA decreasing the learning rate, if requested
"""

#exponentially descrease the learning rate if toggled
def anneal(anneal_lr, update_step, num_update, learning_rate):
  if anneal_lr:
    frac = 1.0 - (update_step - 1.0) / num_update
    lrnow = frac * learning_rate
    return lrnow
  else:
    return learning_rate

"""## Updating agent
Takes care of updating agent relating lists
"""

def collect_data(envs_v, obs_v, actions_v, logprobs_v, rewards_v,\
           dones_v, values_v, next_obs_v, next_done_v, agent_v,\
                 step_loop, device_v):
  obs_v[step_loop] = next_obs_v
  dones_v[step_loop] = next_done_v

  # ALGO LOGIC: action logic
  with torch.no_grad():
      action, logprob, _, value = agent_v.get_action_and_value(next_obs_v)
      values_v[step_loop] = value.flatten()
  actions_v[step_loop] = action
  logprobs_v[step_loop] = logprob

  # TRY NOT TO MODIFY: execute the game and log data.
  next_obs_v, reward, done, info_v = envs_v.step(action.cpu().numpy())
  rewards_v[step_loop] = torch.tensor(reward).to(device_v).view(-1)
  next_obs_v, next_done_v = torch.Tensor(next_obs_v).to(device_v), torch.Tensor(done).to(device_v)

  return obs_v, actions_v, logprobs_v, rewards_v, dones_v, values_v, next_obs_v, next_done_v, info_v

"""## GAE
General Advantage estimation, basically an algorithm to estimate the advantage function.

The original paper about GAE can be found [here](https://arxiv.org/abs/1506.02438)

"""

def GAE(gae_v, gae_lambda_v, gamma_v, agent_v,\
        values_v, dones_v, rewards_v, next_obs_v, next_done_v,\
        num_steps_v, device_v):
  # bootstrap value if not done
  with torch.no_grad():
    next_value = agent_v.get_value(next_obs_v).reshape(1, -1)
    if gae_v:
        advantages_v = torch.zeros_like(rewards_v).to(device_v)
        lastgaelam = 0
        for t in reversed(range(num_steps_v)):
            if t == num_steps_v - 1:
                nextnonterminal = 1.0 - next_done_v
                nextvalues = next_value
            else:
                nextnonterminal = 1.0 - dones_v[t + 1]
                nextvalues = values_v[t + 1]
            delta = rewards_v[t] + gamma_v * nextvalues * nextnonterminal - values_v[t]
            advantages_v[t] = lastgaelam =\
             delta + gamma_v * gae_lambda_v * nextnonterminal * lastgaelam
        returns_v = advantages_v + values_v
    else:
        returns_v = torch.zeros_like(rewards_v).to(device_v)
        for t in reversed(range(num_steps_v)):
            if t == num_steps_v - 1:
                nextnonterminal = 1.0 - next_done_v
                next_return = next_value
            else:
                nextnonterminal = 1.0 - dones_v[t + 1]
                next_return = returns[t + 1]
            returns_v[t] = rewards_v[t] + gamma_v * nextnonterminal * next_return
        advantages_v = returns_v - values_v

  return returns_v, advantages_v

"""## PPO training loop
Training loop with PPO algorithm.

We added kl divergence, that, in a nutshell, it's a simple way to understand how aggressive the policy updates. Further details about the calculation can be found [here](http://joschu.net/blog/kl-approx.html).

"""

def PPO_train_agent(batch_size, update_epochs, minibatch_size, clip_coef, norm_adv, clip_vloss,\
                ent_coef, vf_coef, max_grad_norm, target_kl, \
                agent_v, optimizer_v,scheduler_v, scheduler_flag,\
                b_obs_v, b_actions_v,b_logprobs_v, b_advantages_v, b_returns_v, b_values_v,\
                checkpoint = False):

  #checkpoint is a bool that decides whether to enable or not checkpoint saving

  # Optimizing the policy and value network
  b_inds = np.arange(batch_size)
  clipfracs = []
  loss = 0

  for epoch in range(update_epochs):
      #print('Starting epoch {} of training'.format(epoch))
      np.random.shuffle(b_inds)
      #calculate ratio
      for start in range(0, batch_size, minibatch_size):
          end = start + minibatch_size
          mb_inds = b_inds[start:end]

          _, newlogprob, entropy, newvalue = agent_v.get_action_and_value(b_obs_v[mb_inds],\
                                                                          b_actions_v.long()[mb_inds])
          logratio = newlogprob - b_logprobs_v[mb_inds]
          ratio = logratio.exp()

          # calculate approx_kl
          with torch.no_grad():
              old_approx_kl = (-logratio).mean()
              approx_kl = ((ratio - 1) - logratio).mean()
              clipfracs += [((ratio - 1.0).abs() > clip_coef).float().mean().item()]

          mb_advantages = b_advantages_v[mb_inds]
          if norm_adv:
              mb_advantages = (mb_advantages - mb_advantages.mean()) / (mb_advantages.std() + 1e-8)

          # Policy loss
          pg_loss1 = -mb_advantages * ratio
          pg_loss2 = -mb_advantages * torch.clamp(ratio, 1 - clip_coef, 1 + clip_coef)
          pg_loss = torch.max(pg_loss1, pg_loss2).mean()

          # Value loss
          newvalue = newvalue.view(-1)
          if clip_vloss:
              v_loss_unclipped = (newvalue - b_returns_v[mb_inds]) ** 2
              v_clipped = b_values_v[mb_inds] + torch.clamp(
                  newvalue - b_values_v[mb_inds],
                  -clip_coef,
                  clip_coef,
              )
              v_loss_clipped = (v_clipped - b_returns_v[mb_inds]) ** 2
              v_loss_max = torch.max(v_loss_unclipped, v_loss_clipped)
              v_loss = 0.5 * v_loss_max.mean()
          else:
              v_loss = 0.5 * ((newvalue - b_returns_v[mb_inds]) ** 2).mean()

          entropy_loss = entropy.mean()
          loss_value = pg_loss - ent_coef * entropy_loss + v_loss * vf_coef

          #get checkpoints before updating
          if checkpoint:
            if loss_value < loss:
              agent_v.checkpoint(epoch)

          loss = loss_value
          optimizer_v.zero_grad()
          loss.backward()
          nn.utils.clip_grad_norm_(agent_v.parameters(), max_grad_norm)
          optimizer_v.step()
          if scheduler_flag:
            scheduler_v.step(loss_value)

      if target_kl is not None:
          if approx_kl > target_kl:
              break


  return v_loss, pg_loss, entropy_loss, old_approx_kl, approx_kl, clipfracs, b_values_v, b_returns_v

"""## Evaluate the agent

"""


def evaluate_agent(agent_v,gym_id, seed, device, beta,\
                   num_episodes = 10, step_evaluation = 500, eval_with_video = True):
                       
  # make a brand new environment and record the video
  evaluation_video = f'video_evaluation'
  env = vectorize_env(gym_id, seed, eval_with_video, evaluation_video, 1, beta) 
  
  #initialize storage lists
  obs = torch.zeros((step_evaluation, 1) + env.observation_space.shape).to(device)
  actions = torch.zeros((step_evaluation, 1) + env.action_space.shape).to(device)
  logprobs = torch.zeros((step_evaluation, 1)).to(device)
  rewards = torch.zeros((step_evaluation, 1)).to(device)
  dones = torch.zeros((step_evaluation, 1)).to(device)
  values = torch.zeros((step_evaluation, 1)).to(device)

  next_obs = torch.Tensor(env.reset()).to(device)

  # list of episodic returns
  ep_return = []

  for episode in range(0, num_episodes):
    # start the episode
    print('Starting episode %d' %episode, end = '\r')
    for step in range(0, step_evaluation):
      obs[step] = next_obs

      # selecting action with the actor
      with torch.no_grad():
          action, logprob, _, value = agent_v.get_action_and_value(next_obs)
          values[step] = value.flatten()
      actions[step] = action
      logprobs[step] = logprob

      # do the action and getting the rewards
      next_obs, reward, done, info = env.step(action.cpu().numpy())
      rewards[step] = torch.tensor(reward).to(device).view(-1)
      next_obs = torch.tensor(next_obs).to(device)
    # getting the return for the episode
    ep_return.append(sum(rewards))

  # burocracy so that he doesn't complain
  ep_return = torch.tensor(ep_return)
  ep_mean, ep_std = torch.mean(ep_return).item(), torch.std(ep_return).item()
  return ep_mean, ep_std
