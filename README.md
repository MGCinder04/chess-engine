# ♟️ ChessEngine

A lightweight **UCI-compatible chess engine** written in modern C++20.

It supports **perft testing, iterative deepening, alpha–beta with negamax, quiescence search, transposition tables, killer moves, history heuristic, null-move pruning, late move reductions, and PV display**.

✅ Runs in **CuteChess GUI** or **CuteChess CLI**  
✅ UCI options: `Hash`, `Threads`, `Move Overhead`  
✅ Supports `go depth N` and `go movetime N`

---

## 🚀 Features
- **Legal move generation** with castling & en passant  
- **Perft** (verified through depth 6 from startpos)  
- **Search**
  - Negamax + alpha–beta pruning
  - Iterative deepening
  - Quiescence search
  - Killer moves & history heuristic
  - Transposition table (Zobrist hashing)
  - Null-move pruning
  - Late move reductions (LMR)
  - Principal variation (PV) storage & display
- **UCI protocol**
  - `uci`, `isready`, `ucinewgame`, `stop`, `quit`
  - Options: `Hash`, `Threads`, `Move Overhead`
  - Commands: `go depth N`, `go movetime N`

---

## 📥 Downloads

- **Engine (ChessEngine.exe)**  
  👉 Latest release: https://github.com/MGCinder04/chess-engine/releases

- **CuteChess GUI / CLI**  
  👉 https://github.com/cutechess/cutechess/releases

- **Stockfish** (reference engine)  
  👉 https://stockfishchess.org/download/

- **Lc0 (Leela Chess Zero)** (optional)  
  👉 https://lczero.org/play/download/

---

## 🛠️ Building from Source

### Requirements
- C++20 compiler (MSVC, Clang, or GCC)
- CMake
- Ninja (optional, for faster builds)

### Build (generic)
```bash
git clone https://github.com/MGCinder04/chess-engine.git
cd chess-engine
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```
Output:

```
build/ChessEngine.exe
```

---

## ▶️ Running

### UCI Mode (standalone test)

Run manually:

```bash
./ChessEngine.exe
```

Example session:

```
uci                               # expect: id ... / uciok + options
isready                           # expect: readyok
position startpos
go depth 4                        # expect: info depth 1..4 ... ; bestmove <move>
go movetime 1000                  # search for ~1s, then bestmove
stop                              # interrupts search
quit
```

### Perft Tests

```
position startpos
go perft 1    # expect: 20
go perft 2    # expect: 400
go perft 3    # expect: 8902
```

---

## 🎮 Using with GUI

### CuteChess GUI

1. Open CuteChess **Tools → Settings → Engines → Manage… → New**
2. Browse to `ChessEngine.exe`
3. Protocol = **UCI**
4. Set time control (fixed depth or movetime)
5. Play engine vs engine or human

### CuteChess CLI (example)

Run a quick self-play match:

```bash
cutechess-cli -engine name=rockx cmd=build/ChessEngine.exe proto=uci \
              -engine name=stockfish cmd=stockfish.exe proto=uci \
              -each depth=6 tc=inf \
              -rounds 2 -games 2 -repeat -pgnout match.pgn
```

---

## 📊 Example Output

```
go movetime 1000
info depth 1 score cp 0 nodes 81 time 0 nps 0 pv c2c4
info depth 2 score cp 0 nodes 367 time 4 nps 91750 pv c2c4 c7c5
info depth 3 score cp 0 nodes 3398 time 35 nps 97085 pv h2h3 g8h6 e2e4
...
bestmove e2e4
```

---

## 👨‍💻 Author

Built for learning and portfolio purposes by **Mohit Gupta**.


