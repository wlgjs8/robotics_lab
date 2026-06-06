Goal:
Train, evaluate, and compare imitation-learning policies for the bimanual PIKA UMI bolt pick-place task in robotics_lab.

Task scenario:

1. Right arm picks the black bolt.
2. Right arm places it into the right box.
3. Left arm picks the gray bolt.
4. Left arm places it into the left box.

The HDF5 episodes do not explicitly store phase labels. The right-arm place phase ends before the left-arm phase begins. Phase labels may be inferred only for analysis, metrics, loss weighting, or auxiliary supervision. The policy must not use frame index, episode id, filename number, or timestamp as an input shortcut.

Data discovery:

1. Do not hard-code the number of episodes.

2. Discover valid HDF5 episodes recursively under:

   /home/plaif/workspace/robotics_lab/data

3. Treat subdirectories under this path as session folders.

4. Scan all valid `.hdf5` and `.h5` episodes under the session folders.

5. Exclude obvious non-episode files such as audit, checkpoint, temp, partial, tmp, conversion reports, and manifest files.

6. Run an HDF5 audit before training and reject unsupported or corrupted episodes.

7. Materialize a dataset snapshot before every official experiment:

   * sorted episode paths
   * session folder name
   * sha256 hash
   * frame count
   * detected cameras
   * detected format
   * timestamp range if available
   * action/proprio dimensions
   * audit warnings

8. Save this snapshot as an immutable artifact. Training must use this snapshot, not a live directory scan during training.

Split policy:

1. Use fixed validation ratio 0.2.
2. Split at episode level, never at frame/sample/chunk level.
3. Prefer session-stratified episode split:

   * for each session folder, deterministically assign about 20% of valid episodes to validation
   * remaining episodes go to train
4. If sessions are too small for session-stratified split, fall back to deterministic global episode-level split and report the fallback.
5. Additionally, if enough sessions exist, create an optional stricter `session_holdout_val` split where entire session folders are held out.
6. Save split manifest with:

   * train episode list
   * validation episode list
   * optional session-holdout validation list
   * split seed
   * split algorithm
   * snapshot hash
7. Compute normalization statistics from train episodes only.
8. Do not use validation episodes for training, normalization, crop tuning, threshold tuning, architecture selection by manual inspection, or pretrained adapter fitting.

Current action/state contract:

1. Preserve current policy_runner compatibility.
2. Default action representation should remain the current 14D bimanual action chunk:

   * left dx, dy, dz, drx, dry, drz, gripper
   * right dx, dy, dz, drx, dry, drz, gripper
3. Proprio/state should remain compatible with the current runtime policy_runner format.
4. Any alternative action representation must include an adapter back to the current runtime contract and must be reported separately.

Required baselines:

1. zero-action predictor
2. train-mean action predictor
3. state-only MLP
4. current FlowMatchingPolicy image+state baseline
5. current FlowMatchingPolicy with at least one stronger visual backbone
6. at least one non-flow action-chunk baseline, preferably ACT-style or direct BC chunk regression

Model search space:
The agent may freely experiment, but must organize experiments into comparable families.

Family A: Current flow-matching baseline

* Keep the current rectified-flow action chunk policy.
* Compare tiny_cnn, resnet18, resnet50, and DINO-family frozen backbone if available.
* Compare MLP vs Transformer condition encoder.
* Compare frozen vs last-block-finetuned visual encoder.
* Compare action horizons such as 8, 16, 32, and 48.
* Compare inference sample steps such as 4, 8, 16, and 32.
* Report inference latency.

Family B: Direct behavior cloning action chunk head

* Same image/proprio encoder as flow baseline.
* Predict the whole action chunk directly.
* Compare MSE, L1, SmoothL1/Huber, and normalized per-dimension losses.
* Use this as a sanity check because it is simple and fast.

Family C: ACT-style Transformer action chunking

* Implement or integrate an ACT-like chunk predictor.
* Use multi-view image tokens, proprio tokens, and action-query tokens.
* Output future action chunks.
* Use temporal/action ensembling at inference if implemented.
* Compare against flow baseline under the same train/validation split.

Family D: Diffusion Policy-style action head

* Use the same visual/proprio conditioning as the flow model when possible.
* Implement a conditional action denoiser over `[B, horizon, action_dim]`.
* Compare epsilon-prediction, v-prediction, or x0-prediction if practical.
* Use receding horizon execution.
* Compare inference steps and latency.
* Keep model size comparable to flow baseline before trying larger models.

Family E: Arm-structured heads

* Compare a single 14D bimanual head against structured heads:

  1. shared trunk + separate left/right action heads
  2. shared trunk + arm tokens + cross-arm attention
  3. phase-aware or mixture-of-experts head
  4. gripper-specific head
* For this task, prioritize separate arm heads and gripper-specific losses because the task is mostly sequential: right arm first, then left arm.
* Do not hard-code the task phase as an input unless the phase is available online in a deployable way.
* Phase labels inferred from demonstrations may be used for auxiliary loss and validation metrics.

Family F: Vision encoder and crop ablations

* Compare full resize vs crop.
* Try image sizes such as 128, 224, and 384.
* Compare color-only, depth-only, and color+depth if depth is reliable.
* Compare left camera only, right camera only, and both cameras.
* If using DINOv2/DINOv3/ViT/ConvNeXt, default to frozen backbone plus small adapter first.
* Only finetune visual backbone after frozen-backbone results are logged.

Task-aware loss and metrics:

1. Report normalized action chunk MSE.
2. Report translation endpoint error.
3. Report rotation endpoint error.
4. Report gripper close/open timing error.
5. Report inactive-arm leakage:

   * left arm should not move significantly during right-arm-only phase
   * right arm should not move significantly during left-arm-only phase unless demonstrations show otherwise
6. Report phase-weighted metrics:

   * right pick
   * right place
   * left pick
   * left place
7. Add optional gripper-event weighting because gripper close/open is sparse.
8. Add optional inactive-arm penalty.
9. Add optional smoothness penalty across predicted action chunks.
10. Always report unweighted metrics too.

Shortcut prevention:

1. Do not use frame index, timestamp, episode id, filename, or session id as model input.
2. Do not use validation data for normalization.
3. Do not split samples from the same episode across train and validation.
4. Do not tune crop coordinates or thresholds using validation videos.
5. Do not cherry-pick only successful runs.
6. Report failed runs and unstable runs.
7. Run image-shuffle and zero-image ablations:

   * if performance barely changes, report that the policy may not be visually grounded.
8. Run state-only baseline:

   * if state-only matches image+state, report that the task may be mostly solvable from proprio/state trajectory priors.

Success criteria:

1. The best learned model must outperform zero-action and train-mean baselines by a large margin on the fixed validation split.
2. The best image+state model should outperform state-only baseline.
3. If image+state does not outperform state-only, do not claim visual grounding.
4. The best model must report:

   * validation metrics
   * train metrics
   * model config
   * dataset snapshot hash
   * split hash
   * checkpoint hash
   * GPU usage
   * inference latency
5. No validation leakage is allowed.
6. The final answer must include a leaderboard across model families and an explanation of the recommended next rollout candidate.

GPU usage:

1. The GPU server has 8 GPUs available.
2. Use them freely, but efficiently.
3. Prefer parallel single-GPU sweeps for small/medium models.
4. Use DDP only when model size or batch size justifies it.
5. If using DDP, implement correct DistributedSampler, per-rank seed handling, rank-zero checkpointing, and synchronized metric aggregation.
6. Log actual GPU ids and utilization.

Implementation deliverables:

1. Dataset snapshot generator for `/home/plaif/workspace/robotics_lab/data`.
2. Fixed episode-level split manifest generator with val_ratio=0.2.
3. Train-only normalization statistics.
4. Updated training code that accepts dataset snapshot and split manifest.
5. Baseline evaluators:

   * zero-action
   * train-mean
   * state-only MLP
   * direct BC chunk regression
   * current flow matching
6. At least one stronger action-head family:

   * ACT-style, Diffusion Policy-style, or structured arm-head flow model
7. Evaluation report:

   * leaderboard
   * metrics by phase
   * shortcut ablations
   * model configs
   * checkpoint hashes
   * dataset/split hashes
8. Clear recommendation for the next model to test in policy_runner rollout.

Recommended first experiment order:

1. Dataset audit and fixed split.
2. zero-action, train-mean, state-only MLP.
3. current flow baseline with tiny_cnn/resnet18.
4. direct BC chunk regression.
5. flow baseline with frozen ResNet50 or DINO-family encoder.
6. structured arm-head flow model.
7. ACT-style chunk policy.
8. Diffusion Policy-style action head.
9. Higher image resolution and crop ablations.
10. Optional session-holdout validation if enough sessions exist.
