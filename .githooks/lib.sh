#!/usr/bin/env bash
# 规约层共享规则库

ALLOWED_TYPES=(feat fix perf docs style refactor test build ci chore revert merge)
SCOPE_WHITELIST=()
SUBJECT_MAX_LEN=72
NO_TRAILING_PERIOD=1
BODY_LINE_MAX_LEN=100

BRANCH_PATTERNS=(
  'main'
  'develop'
  'release/[a-z0-9._-]+'
  '(feat|fix|perf|refactor|test|docs|chore)/[a-z0-9._/-]+'
)

FORBIDDEN_PATHS=(^build/ ^install/ ^log/ ^__pycache__/)
MAX_FILE_SIZE=$((5 * 1024 * 1024))
CHECK_CONFLICT_MARKERS=1

warn()  { echo >&2 "⚠ $*"; }
err()   { echo >&2 "✘ $*"; }
hint()  { echo >&2 "  $*"; }

str_width() {
  python3 -c 'import sys;print(sum(2 if ord(c)>0x2E80 else 1 for c in sys.argv[1]))' "$1"
}

in_array() {
  local v="$1"; shift
  local x
  for x in "$@"; do [ "$x" = "$v" ] && return 0; done
  return 1
}

check_subject() {
  local s="$1"

  if ! echo "$s" | grep -Eq '^(feat|fix|perf|docs|style|refactor|test|build|ci|chore|revert|merge)(\([a-zA-Z0-9_/-]+\))?!?: .'; then
    err "不符合 <type>(<scope>): <subject> 格式: $s"
    hint "type: ${ALLOWED_TYPES[*]}"
    hint "示例: feat(backend): 地图落盘 + 逐点 deskew"
    return 1
  fi

  if [ "$NO_TRAILING_PERIOD" = 1 ] && echo "$s" | grep -Eq '[。.]$'; then
    err "subject 结尾不要加句号: $s"
    return 1
  fi

  local w
  w="$(str_width "$s")"
  if [ "$w" -gt "$SUBJECT_MAX_LEN" ]; then
    err "subject 过长(${w} > ${SUBJECT_MAX_LEN} 显示宽度): $s"
    return 1
  fi

  return 0
}
