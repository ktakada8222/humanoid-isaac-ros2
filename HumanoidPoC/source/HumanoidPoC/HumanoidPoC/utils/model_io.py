# model_io.py
import onnxruntime as ort
import numpy as np

def load_onnx_model(policy_path: str, encoder_path: str | None = None):
    """ONNX ポリシー/エンコーダの InferenceSession を作成して返す。"""
    # 必要なら GPU などに切り替え: providers=["CUDAExecutionProvider","CPUExecutionProvider"]
    policy_sess = ort.InferenceSession(policy_path, providers=["CPUExecutionProvider"])
    encoder_sess = ort.InferenceSession(encoder_path, providers=["CPUExecutionProvider"]) if encoder_path else None
    return policy_sess, encoder_sess

def run_policy(
    obs_policy: np.ndarray,
    command_3: np.ndarray | None,
    encoder_obs: np.ndarray | None,
    policy_sess,
    encoder_sess=None,
    obs_mean: np.ndarray | None = None,
    obs_std: np.ndarray | None = None,
) -> np.ndarray:
    """観測(＋任意のエンコーダ潜在/コマンド)からアクションを推論して返す。"""
    # --- normalize ---
    x_policy = obs_policy.astype(np.float32)
    x_encoder = encoder_obs.astype(np.float32) if encoder_obs is not None else None
    cmd = command_3.astype(np.float32) if command_3 is not None else np.zeros(3, dtype=np.float32)

    if obs_mean is not None and obs_std is not None:
        x_policy = (x_policy - obs_mean) / (obs_std + 1e-8)
        if x_encoder is not None:
            x_encoder = (x_encoder - obs_mean) / (obs_std + 1e-8)

    # --- encoder → latent ---
    if encoder_sess is not None and x_encoder is not None:
        enc_in = x_encoder.reshape(-1)
        z = encoder_sess.run(
            [encoder_sess.get_outputs()[0].name],
            {encoder_sess.get_inputs()[0].name: enc_in},
        )[0]
        pol_in = np.concatenate([z.reshape(-1), x_policy.reshape(-1), cmd], axis=0)
    else:
        pol_in = x_policy  # ポリシー入力に cmd を入れない設計ならこのまま

    # --- policy inference ---
    action = policy_sess.run(
        [policy_sess.get_outputs()[0].name],
        {policy_sess.get_inputs()[0].name: pol_in},
    )[0]
    return action.reshape(-1)
