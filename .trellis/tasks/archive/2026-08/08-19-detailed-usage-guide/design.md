# Detailed Usage Guide Design

## Deliverables

The documentation surface consists of a paired entry point:

```text
README.md
  -> docs/usage-guide.md        English technical baseline
  -> docs/usage-guide.zh-CN.md  Simplified Chinese equivalent
```

Both documents are complete operational guides, not a translation of every
specialized runbook. Their first paragraph links to the other language version
and their final sections link to the existing authoritative details.

## Shared Structure

The two documents use matching sections in this order:

1. Purpose, supported scope, and three safety classes.
2. Repository architecture and prerequisites.
3. Host quick start: configuration validation, architecture check, arm
   simulation, behavior-tree example, and full CI.
4. Configuration model and supported robot YAML examples.
5. Motor-disabled navigation and behavior-tree inspection.
6. Motion runtime and telemetry recording boundary.
7. Orange Pi target preparation and qualification handoff.
8. Physical-motor exclusions, failure response, and related runbooks.

Commands remain literal and are only included after confirming source, script,
CLI or test ownership. Each command block is followed by a concise statement
of scope: host-safe/read-only, target-only/disarmed, or physical gate.

## Safety Model

| Class | Documentation behavior |
|---|---|
| Host-safe/read-only | May appear as an executable quick-start command. Explain that it cannot qualify hardware or realtime behavior. |
| Target-only/disarmed | List prerequisites and link to the authoritative platform runbook. Do not place a privileged mutation command in the quick start. |
| Physical gate | Name the unmet safety fixture, evidence, and ownership requirement. Do not provide a motor enablement sequence. |

The guide states that candidate profiles cannot be promoted by a host result,
that 1 kHz needs immutable target evidence, and that a failed 1 kHz profile
does not silently select 500 Hz.

## Ownership and Links

The guide owns navigation between supported workflows. It does not duplicate
the detailed platform or domain procedures owned by:

- `platform/orangepi5plus/docs/{build-and-install,bring-up,realtime-qualification,rollback}.md`;
- `docs/{behavior-tree,navigation-plugin,arm-payload-identification}.md`; and
- `platform/orangepi5plus/README.md`.

The README receives a concise bilingual documentation link near the existing
configuration and runbook links. Relative links are used so the rendered files
work from a source checkout and Git hosting view.

## Synchronization Rule

English is the baseline for technical identifiers and source-facing wording.
Every section, command block, warning category and outbound link has a
corresponding Chinese entry. Translate narrative only; preserve code fences,
paths, package names, flags, YAML keys and evidence labels byte-for-byte.

## Rollback

The change is documentation-only. Revert the README links and the paired guide
files together. No runtime, platform, profile, hardware or evidence state is
changed.
