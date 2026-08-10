# ELF3 SONIC step28800 modular ONNX export

These are the standard 10-frame modular deployment artifacts exported from the
same ELF3 SONIC step28800 checkpoint as the runtime SMPL policy. No
low-latency checkpoint or four-frame configuration is included.

## Official module structure

- `model_step_028800_encoder.onnx` contains the `g1`, three-point `teleop`, and
  full-body `smpl` encoders, the FSQ stage, and a scalar mode selector:
  `obs_dict[1,1751] -> encoded_tokens[1,64]`.
- `model_step_028800_decoder.onnx` is the deployed `g1_dyn` action decoder:
  `obs_dict[1,994] -> action[1,29]`.

The decoder input is the 64-D SONIC motion token followed by the live 930-D
ELF3 proprioceptive history. An external GR00T 64-D token therefore bypasses
the encoder and is concatenated directly with that history before decoder
inference. The external token must use the same step28800 SONIC/FSQ latent
semantics; matching only the tensor shape is not sufficient.

The official exporter does not produce three independent encoder files and two
deployment decoder files. It produces one multi-mode encoder and one `g1_dyn`
decoder. The checkpoint's `g1_kin` decoder is a training-only kinematic
reconstruction head. The fixed-mode `g1`, `teleop`, and `smpl` exports are
end-to-end convenience policies, not standalone encoders. The already-used
SMPL convenience policy remains at
`../elf3_step28800_smpl/model_step_028800_smpl.onnx`.

## Publication integrity

Only ONNX node `doc_string` fields containing export-machine stack traces were
removed; graph parameters and inference contracts are unchanged. The original
checkpoint SHA-256 is
`d40f080015aef9f36975eea79b23e376186fc60c24df6654fe0cbf8b5bad2b6b`.

Cleaned artifact SHA-256 values:

```text
8b30e2f9afc24e081662ea8daf47724753d16918e633688c9561ef4cc54d71e0  model_step_028800_encoder.onnx
8f60f1aae1191ac22a8bb1b0eaf98110529adaf09d2a13bb6d9d61c873bbf69f  model_step_028800_decoder.onnx
```

The weights are governed by the NVIDIA Open Model License in
`../../../../../third_party/GEAR_SONIC_LICENSE.txt`; repository-level
attribution is in `../../../../../THIRD_PARTY_NOTICES.md`.
