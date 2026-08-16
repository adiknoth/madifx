savedcmd_madifx.mod := printf '%s\n'   madifx.o | awk '!x[$$0]++ { print("./"$$0) }' > madifx.mod
