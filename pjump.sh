# Add this to your ~/.bashrc or ~/.zshrc
# Usage: pjump my-project-folder

pjump() {
    local BASE="C:\Users\Jimg\Documents\PlatformIO\Projects"
    local TARGET="$BASE/$1"
    if [ -d "$TARGET" ]; then
        cd "$TARGET"
        echo "🔁 Switched to: $(pwd)"
        ls
    else
        echo "❌ Project not found at $TARGET"
    fi
}
