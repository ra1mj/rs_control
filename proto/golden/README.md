# Protobuf compatibility baseline

`robot-v1.binpb` is a deterministic `google.protobuf.FileDescriptorSet` produced by:

```bash
tools/bufw build --as-file-descriptor-set -o proto/golden/robot-v1.binpb
```

Check source compatibility before replacing it:

```bash
tools/bufw breaking --against proto/golden/robot-v1.binpb
```

The baseline is replaced only after an additive v1 contract change has passed review. Breaking semantics require a new package/key-space version.
