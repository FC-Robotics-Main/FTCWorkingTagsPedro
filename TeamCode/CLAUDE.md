# Master CLAUDE.md — SCH Standard Workflow
# Location: D:\Nextcloud.SCHOffice\Main.SCH\Claude-All\CLAUDE.md
# Applies to: every project under Claude-All (loaded automatically when working in any subfolder)

## Who I am
- Solo operator on Windows 10 or 11. All Claude Code / Cowork projects live under
  `D:\Nextcloud.SCHOffice\Main.SCH\Claude-All\` ("the project root").
- This folder tree is synced by Nextcloud. Assume every file write is replicated
  to other machines.

## Autonomy
- Work autonomously. Do not stop mid-task to ask whether you may read, create,
  edit, or run something within the current project folder. Make the reasonable
  choice, state the assumption in your output, and keep going.
- Only stop and ask when: (a) an action would delete or overwrite work that
  cannot be recreated, (b) an action reaches outside the current project folder,
  or (c) the request is genuinely ambiguous about *what* to build, not *how*.
- Never ask multiple setup questions in a row. Batch any truly necessary
  questions into one message.

## New project setup (Claude Code or Cowork)
When a session starts in `Claude-All\` itself, or I say "new project":
1. Ask me for the project name (one question, nothing else).
2. Create `D:\Nextcloud.SCHOffice\Main.SCH\Claude-All\<Project-Name>\` using
   Title-Case-With-Hyphens.
3. Inside it, create this skeleton:
   - `reports\`
   - `PROMPTS.md`        (running prompt log — see below)
   - `TRANSFER.md`       (handoff file — see below)
   - `CLAUDE.md`         (project-level extension; start from the template at
     `Claude-All\_templates\CLAUDE-project.md` if present, otherwise minimal)
4. Confirm the path back to me in one line, then proceed with the first task.

## Reports (all substantive responses)
- Every substantive deliverable (analysis, research, design doc, plan, memo)
  is written as a Markdown file in `<project>\reports\`, not only in chat.
  Give a 3–6 line summary in chat and link the file.
- File naming: `YYYY-MM-DD-Short-Description-V01.md`
  (example: `2026-07-05-Foundation-Cost-Analysis-V01.md`).
- Never edit a report in place. Any change = save a new file with the version
  incremented (`-V02`, `-V03`, ...). The date in the filename stays the date
  the report was first created; only the version changes.
- Trivial conversational replies (yes/no answers, quick clarifications) do not
  need a report file.

## PROMPTS.md (per project, not versioned)
- Single file at the project root. Append-only, updated every turn.
- Each entry: `## YYYY-MM-DD HH:MM` followed by my prompt verbatim (or a
  faithful condensation if it included large pasted content — note what was
  attached).
- Never version this file. Just update it.

## TRANSFER.md (per project, not versioned)
- Single file at the project root, rewritten/updated at the end of every turn.
- Purpose: a cold-start briefing so a different AI agent (or a fresh Claude
  session on another account) can take over with zero prior context.
- Required sections:
  1. **Project goal** — one paragraph.
  2. **Current state** — what exists, what works, what is in progress.
  3. **Key decisions & constraints** — with brief reasons.
  4. **File map** — the important files and what each one is.
  5. **Open items / next steps** — ordered.
  6. **Gotchas** — anything a new agent would trip over.
- Keep it under ~2 pages. It replaces prior contents; it is a snapshot, not a log.

## Writing & output standards
- All documents in Markdown unless I ask otherwise.
- Define jargon in plain language the first time it appears.
- Lead with the strategic answer/verdict, then supporting detail.
- For research/legal/financial analysis: end with a "Questions for my
  attorney/CPA/engineer" list where relevant.
- No emojis in files or code comments.

## Code & environment conventions
- Windows paths with backslashes in docs; use forward slashes only inside code
  where the language requires it.
- Prefer PowerShell for scripts I will run manually.
- Java projects: target the FTC toolchain (Liberica JDK 17 Full, Android Studio
  conventions) unless stated otherwise.
- Git: initialize a repo in each new project folder; commit at meaningful
  checkpoints with clear messages. Never force-push. Add `node_modules/`,
  build output, and other bulk artifacts to `.gitignore` (they also bloat
  Nextcloud sync — keep them out of synced state where possible).
## Never delete — always Archive
- NEVER delete any file or folder, under any circumstances, even if I ask you
  to "delete" something. "Delete" always means: move it to
  `<project>\Archive\YYYY-MM-DD\`, preserving its relative path so nothing
  collides. Create the dated subfolder if it doesn't exist.
- The same applies to overwrites of non-report files: before replacing an
  existing file's contents wholesale, move the old copy into Archive first.
  (Reports are already protected by versioning — new version file, never edit
  in place.)
- Never empty, prune, or "clean up" the Archive folder itself. It is
  append-only. If it gets large, tell me and I will handle it manually.
- Exception: files Claude itself created within the current session as
  throwaway scratch (temp scripts, intermediate build output) may be removed
  without archiving — but when in doubt, archive.

## Context window management (ongoing)
- At natural checkpoints (a task finishes, a report is saved, before starting a
  new major task), add one short line at the end of your response estimating
  how full the context window is (rough: fresh / getting long / near limit)
  and whether now is a good time to compact.
- When you judge the session is "getting long," recommend I run `/compact`
  BEFORE starting the next task, and propose the compact instruction to use,
  e.g.: `/compact keep the report naming rules, current file map, and open
  items; drop tool output and superseded drafts`.
- ALWAYS update TRANSFER.md immediately before any compaction (or when you
  warn that auto-compact is near). TRANSFER.md is the ground truth that
  survives if compaction loses detail.
- If you notice you have lost earlier context (I reference something you no
  longer have), say so plainly and re-read TRANSFER.md and PROMPTS.md instead
  of guessing.

## Boundaries
- Never modify files outside the current project folder unless I explicitly
  name the path in my prompt.
- Never touch Nextcloud config, Windows system settings, or credentials.
- If a task seems to require something outside these boundaries, say so and
  propose the in-bounds alternative.
- Reaching outside the project folder triggers an approval prompt every time,
  even in acceptEdits mode. Do the work inside the project folder instead.
  The project CLAUDE.md and the project .claude\settings.local.json are
  in-bounds equivalents of the parent-level files.

## Never ask questions
- Never end a turn with a question or a menu of options. Pick the sensible
  default, do the work, and state the assumption in one line at the end.
- No "Questions for you" or "Questions for my attorney/CPA/engineer" sections
  in reports for this project.
- Never use the AskUserQuestion tool.
- Batch nothing, defer nothing, wait for nothing. Finish the work.
- Write commands that match existing allow rules so they do not trigger
  approval dialogs. An approval dialog is a question. Never put destructive
  flags (-Recurse -Force, rm -rf, --force) in a command; archive instead.

## Words never to use
- Never say "let me verify", "let me check", "instead of guessing", or any
  variant. Verification is not an announcement. Do it silently and report only
  the verified result. Never guess, and never narrate the absence of guessing.
- Never invent shorthand or labels (M0, M1, phase names) and then use them as
  if they were shared vocabulary. Say what the step actually is in plain words.
- Define jargon the first time it appears, every time.
- Spinner verbs: only "Thinking" and "Working". These are set in
  .claude\settings.local.json under spinnerVerbs, not here, because the spinner
  is drawn by the harness and never reads CLAUDE.md.
