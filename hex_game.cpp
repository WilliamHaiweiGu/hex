/**
 * Single player hex game.
 * @author github.com/WilliamHaiweiGu
 * The game engine uses Alpha-Beta Prune tree search with paralle Monte-Carto method for board evaluation.
 * Change parameters in main function for boards of other sizes or if engine is taking more than 2 minutes to make a
 * move.
 *
 * [IMPORTANT]
 * Check your G++ version by running "g++ --version". This program uses semaphores and requires G++ 8.0 or later to
 * compile. Compile and run with "g++ -std=c++20 -O3 hex_game.cpp && a.exe".
 * What if version is lower? Use the most recent version of Cygwin and use its g++ or use the most recent version of
 * WSL (update by running "sudo apt update && sudo apt upgrade && sudo apt dist-upgrade && sudo do-release-upgrade" in
 * WSL) and use its g++.
 */

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <list>
#include <random>
#include <semaphore>
#include <thread>
#include <vector>

using namespace std;

enum hex_status : char
{
    blk = 'X',
    wht = 'O',
    non = '.'
};

ostream &operator<<(ostream &os, hex_status p)
{
    os << static_cast<char>(p);
    return os;
}

/** White/first-hand player: bottom to top. Black/second-hand player: left to right*/
class hex_graph
{
public:
    /** Side length of board*/
    const unsigned int len;
    /** Always equal to len*len*/
    const unsigned int mat_size;
    /**
     * @param len side length of board
     */
    hex_graph(unsigned int len) : len(len), n1(len - 1), mat_size(len * len)
    {
        const unsigned int n_pos = len * len;
        board = new hex_status[n_pos];
        for (int i = 0; i < n_pos; i++)
            board[i] = non;
        empty_pos.reserve(n_pos);
    }
    ~hex_graph()
    {
        delete[] board;
    }
    void print(ostream &os) const
    {
        for (unsigned int i = 0;; i++)
        {
            for (unsigned int j = 0; j < i; j++)
                os << "  ";
            os << get(i, 0);
            for (unsigned int j = 1; j < len; j++)
                os << " - " << get(i, j);
            if (i >= n1)
                break;
            os << '\n';
            for (unsigned int j = 0; j < i; j++)
                os << "  ";
            os << " \\";
            for (unsigned int j = 1; j < len; j++)
                os << " / \\";
            os << '\n';
        }
    }
    /** Update board with specified 1d index. No bounds checks.*/
    inline void set(unsigned int idx1d, hex_status val)
    {
        board[idx1d] = val;
    }
    /** No bounds checks.*/
    inline void set(unsigned int i, unsigned int j, hex_status val)
    {
        *ptr_of(i, j) = val;
    }
    /** No bounds checks.*/
    inline hex_status get(unsigned int i, unsigned int j) const
    {
        return *ptr_of(i, j);
    }
    /** No bounds checks*/
    inline bool is_legal_move(unsigned int i, unsigned int j) const
    {
        return get(i, j) == non;
    }
    /** @return Bounds check*/
    inline bool is_in_range(unsigned int idx) const
    {
        return idx < len;
    }
    /**
     * @return wht if white wins, blk if black wins, non if ongoing
     */
    hex_status game_status() const
    {
        bool *visited = new bool[mat_size];
        if (white_wins(visited))
        {
            delete[] visited;
            return wht;
        }
        // get ready for dfs
        for (unsigned int k = 0; k < mat_size; k++)
            visited[k] = false;
        // try find black path
        for (int i = 0; i < len; i++)
            if (dfs(i, 0, visited, blk))
            {
                delete[] visited;
                return blk;
            }
        // No path exists
        delete[] visited;
        return non;
    }

    /** Count current empty positions and store internally*/
    void list_empty_pos()
    {
        empty_pos.clear();
        empty_pos.reserve(mat_size);
        for (unsigned int i = 0; i < mat_size; i++)
            if (board[i] == non)
                empty_pos.push_back(i);
    }

    /**
     * @return Pointer to the lastest computation result of list_empty_pos()
     */
    inline const vector<unsigned int> *get_empty_pos() const
    {
        return &empty_pos;
    }

    /**
     * Create a string that represents the board data.
     * @return a new string of the board data.
     */
    inline string to_string() const
    {
        return string(reinterpret_cast<char *>(board), mat_size);
    }

    friend class monte_carlo_simulator;

private:
    /** Always equal to len-1*/
    const unsigned int n1;
    /** Only used in list_empty_pos()*/
    vector<unsigned int> empty_pos;
    /** 2d matrix of hex_status*/
    hex_status *board;
    /**
     * @param i row index
     * @param j column index
     * @return Respective position in 1d representation of matrix
     */
    inline unsigned int idx1d_of(unsigned int i, unsigned int j) const
    {
        return i * len + j;
    }
    /**
     * @param idx_id index in 1d array
     * @return Row index of item in matrix
     */
    inline unsigned int row_of(unsigned int idx_1d) const
    {
        return idx_1d / len;
    }
    /**
     * @param idx_id index in 1d array
     * @return Column index of item in matrix
     */
    inline unsigned int col_of(unsigned int idx_1d) const
    {
        return idx_1d % len;
    }
    /**
     * No bounds check
     * @return pointer to position on hex board
     */
    inline hex_status *ptr_of(unsigned int i, unsigned int j) const
    {
        return &board[idx1d_of(i, j)];
    }

    /**
     * @param visited the 1d buffer for visited matrix. Overwrites its content.
     * @return whether white side has won the game
     */
    bool white_wins(bool *visited) const
    {
        // get ready for dfs
        for (unsigned int k = 0; k < mat_size; k++)
            visited[k] = false;
        // try find white path
        for (int j = 0; j < len; j++)
            if (dfs(n1, j, visited, wht))
                return true;
        return false;
    }

    /**
     * DFS helper. Push to stack if unvisited
     * @param i row index
     * @param j col index
     * @param visited 2d bool matrix, positions visited at the point
     * @param stack positions scuduled to visit at the point
     * @param side the searching side
     */
    void visit(unsigned int i, unsigned int j, bool *visited, list<unsigned int> &stack, const hex_status side) const
    {
        const unsigned int idx1d = idx1d_of(i, j);
        if (get(i, j) == side && !visited[idx1d])
        {
            visited[idx1d] = true;
            stack.push_back(idx1d);
        }
    }

    /**
     * Find a path from starting position to any position satisfying pred.
     * @param i0 starting row index
     * @param j0 starting col index
     * @param visited 2d bool matrix, positions visited at start
     * @param side the searching side. Must be wht or blk
     * @return whether a path exists
     */
    bool dfs(const unsigned int i0, const unsigned int j0, bool *visited, const hex_status side) const
    {
        list<unsigned int> stack; // Stack of 1d idx
        visit(i0, j0, visited, stack, side);
        const bool is_white_side = side == wht;
        while (!stack.empty())
        {
            const unsigned int cur_1d = stack.back();
            const unsigned int i = row_of(cur_1d);
            const unsigned int j = col_of(cur_1d);

            if (is_white_side ? i <= 0 : j >= n1) // this can be replaced by any search target condition
                return true;
            stack.pop_back();
            // neighbors
            const unsigned int i_prev = i - 1;
            const unsigned int j_next = j + 1;
            const bool has_top = i > 0;
            const bool has_right = j < n1;
            if (has_right)
                visit(i, j_next, visited, stack, side);
            if (has_right && has_top)
                visit(i_prev, j_next, visited, stack, side);
            if (has_top)
                visit(i_prev, j, visited, stack, side);
            const unsigned int i_next = i + 1;
            const unsigned int j_prev = j - 1;
            const bool has_bottom = i < n1;
            const bool has_left = j > 0;
            if (has_left)
                visit(i, j_prev, visited, stack, side);
            if (has_left && has_bottom)
                visit(i_next, j_prev, visited, stack, side);
            if (has_bottom)
                visit(i_next, j, visited, stack, side);
        }
        return false;
    }
};

ostream &operator<<(ostream &os, hex_graph &hex)
{
    hex.print(os);
    return os;
}

/** Runs Monte-Carlo trials on given boards to eval*/
class monte_carlo_simulator
{
public:
    /** Number of trials ran by this simulator*/
    const unsigned int n_trial;
    /** Board reused by this simulator*/
    hex_graph sim_board;
    /** DFS visited matrix*/
    bool *visited;
    monte_carlo_simulator(unsigned int n_trial, unsigned int len) : n_trial(n_trial), sim_board(len), rd_gen((random_device())())
    {
        visited = new bool[sim_board.mat_size];
    }
    ~monte_carlo_simulator()
    {
        delete[] visited;
    }
    /**
     * Update board data
     * @param other_board pointer to the other board
     */
    inline void set(hex_graph *board)
    {
        this->board = board;
        empty_pos = board->get_empty_pos();
    }
    /**
     * Run the simulation.
     * @return number of trials where white wins.
     */
    void run()
    {
        const unsigned int n_empty_pos = empty_pos->size();
        const unsigned int n_white_pos = n_empty_pos / 2 + (1 & n_empty_pos & sim_board.mat_size); // add 1 iff board size and empty size are both odd
        white_win_cnt = 0;
        memcpy(sim_board.board, board->board, sizeof(hex_status) * sim_board.mat_size);
        vector<unsigned int> empty_pos_rand = *empty_pos;
        for (unsigned int i_trial = 0; i_trial < n_trial; i_trial++)
        {
            shuffle(empty_pos_rand.begin(), empty_pos_rand.end(), rd_gen);
            unsigned int i = 0;
            for (; i < n_white_pos; i++)
                sim_board.board[empty_pos_rand[i]] = wht;
            for (; i < n_empty_pos; i++)
                sim_board.board[empty_pos_rand[i]] = blk;
            const bool res = sim_board.white_wins(visited);
            if (res)
                white_win_cnt++;
        }
    }
    /**
     * @return number of trials where white wins.
     */
    inline unsigned int get_white_win_count()
    {
        return white_win_cnt;
    }

private:
    default_random_engine rd_gen;
    /** White side's winning count in the latest simulation*/
    unsigned int white_win_cnt;
    /** The starting board to simulate*/
    hex_graph *board;
    /** Latest computed empty positions*/
    const vector<unsigned int> *empty_pos;
};

/** Runs fixed numbers of Monte Carlo simulations asynchronously. Can run multiple times with different board data without recreating threads.*/
class monte_carlo_async_worker
{
public:
    /**The underliying simulator*/
    monte_carlo_simulator sim;
    thread t;
    monte_carlo_async_worker(unsigned int n_trial, unsigned int len) : sim(n_trial, len), stop(false)
    {
        t = thread(&monte_carlo_async_worker::thread_fn, this);
    }

    void thread_fn()
    {
        while (true)
        {
            working.acquire();
            if (stop)
                break;
            sim.run();
            waiting.release();
        }
    }

    /** Must not run twice without calling get_res() in between. Must be paired with a get_res() after*/
    inline void start()
    {
        working.release();
    }

    /**
     * Blocks until simulation is done. Must not run twice without calling get_res() first.
     * @return number of trials where white wins.
     */
    inline unsigned int get_res()
    {
        waiting.acquire();
        return sim.get_white_win_count();
    }

    /**
     * Must run before start() or after get_res().
     * @param other_board pointer to the other board
     */
    inline void set(hex_graph *other_board)
    {
        sim.set(other_board);
    }

    ~monte_carlo_async_worker()
    {
        stop = true;
        start();
        t.join();
    }

private:
    /** Controls start of computation*/
    binary_semaphore working{0};
    /** Controls when it's safe to get computation result/update simulator input*/
    binary_semaphore waiting{0};
    /** Whether this worker is instructed to stop*/
    bool stop;
};

/** Makes best move using Alpha-Beta prune algorithm.*/
class game_engine
{
public:
    static constexpr int WHITE_WINS_EVAL = INT_MAX - 1;
    static constexpr int BLACK_WINS_EVAL = INT_MIN + 1;
    hex_graph &board;
    /** Max deapth of search tree*/
    const unsigned int depth;
    /** Number of Monte-Carlo trials to do in each thread*/
    const unsigned int each_n_trials;

    /** Inirilize game engine
     * @param board The initial board.
     * @param depth Search depth. Must be >0.
     * @param n_trials Number of trials in each Monte-Carlo simulation.
     * @param n_threads Number of threads during each board evaluation.
     */
    game_engine(hex_graph &board, unsigned int depth, unsigned int n_trials, unsigned int n_threads) : board(board),
                                                                                                       depth(depth),
                                                                                                       memos(depth),
                                                                                                       each_n_trials((n_trials - 1) / n_threads + 1),
                                                                                                       total_n_trials_half((each_n_trials * n_threads) / 2)
    {
        workers.reserve(n_threads);
        for (unsigned int i = 0; i < n_threads; i++)
            workers.push_back(new monte_carlo_async_worker(each_n_trials, board.len));
    }
    ~game_engine()
    {
        for (monte_carlo_async_worker *worker : workers)
            delete worker;
    }
    /**
     * @return INT_MIN if black wins, INT_MAX if white wins, a number indicating white's advantage otherwise.
     */
    int monte_carlo_eval() const
    {
        const int end_status = end_game_status();
        if (end_status != 0)
            return end_status;
        board.list_empty_pos();
        int ans = -total_n_trials_half;
        for (monte_carlo_async_worker *worker : workers)
        {
            worker->set(&board);
            worker->start();
        }
        for (monte_carlo_async_worker *worker : workers)
            ans += worker->get_res();
        return ans;
    }

    /**
     * Perform an alpha beta prune search from a side's POV. Using property of game: a board position can only occur at one search depth; won't search for more than 4 layers
     * @param side whether the engine is on white side.
     * @return index to put a piece as next move.
     */
    unsigned int alpha_beta_search(bool side)
    {
        for (unordered_map<string, unsigned int> &memo : memos)
            memo.clear();
        return side ? max_search(INT_MIN, INT_MAX, depth).second : min_search(INT_MIN, INT_MAX, depth).second;
    }

private:
    vector<monte_carlo_async_worker *> workers;
    const unsigned int total_n_trials_half;
    /**
     * Compute and give end game eval or 0 it game is ongoing.
     * @return BLACK_WINS_EVAL if black wins, WHITE_WINS_EVAL if white wins, 0 otherwise.
     */
    inline int end_game_status() const
    {
        switch (board.game_status())
        {
        case blk:
            return BLACK_WINS_EVAL;
        case wht:
            return WHITE_WINS_EVAL;
        }
        return 0;
    }

    /** Look up tables by search layer*/
    vector<unordered_map<string, unsigned int>> memos;

    /**
     * Searches white's best move
     * @param a alpha value
     * @param b beta value
     * @param depth depth of this search.
     * @return pair<eval, best move>. The best-move argument is optional: UNSGINED_MAX is not provided
     */
    pair<int, unsigned int> max_search(int a, int b, unsigned int depth)
    {
        if (depth <= 0)
            return make_pair(monte_carlo_eval(), UINT32_MAX);
        const int end_status = end_game_status();
        if (end_status != 0)
            return make_pair(end_status, UINT32_MAX);
        board.list_empty_pos();
        vector<unsigned int> empty_pos = *board.get_empty_pos();

        const unsigned d1 = depth - 1;
        unordered_map<string, unsigned int> &memo = memos[d1];

        int val = INT_MIN;
        unsigned int best_move = UINT32_MAX;
        for (unsigned int pos : empty_pos) // loop invariant: board is identical as if unchanged
        {
            // memoize search result of childs
            board.set(pos, wht);
            auto [iter, tb_miss] = memo.try_emplace(board.to_string());
            int val_next;
            if (tb_miss)
            {
                val_next = min_search(a, b, d1).first;
                iter->second = val_next;
            }
            else
                val_next = iter->second;
            board.set(pos, non);
            // ab prune
            if (val_next > val)
            {
                val = val_next;
                best_move = pos;
                a = max(a, val);
            }
            if (val >= b)
                break;
        }
        return make_pair(val, best_move);
    }

    /**
     * Searches black's best move
     * @param a alpha value
     * @param b beta value
     * @param depth depth of this search.
     * @return pair<eval, best move>. The best-move argument is optional: UNSGINED_MAX is not provided
     */
    pair<int, unsigned int> min_search(int a, int b, unsigned int depth)
    {
        if (depth <= 0)
            return make_pair(monte_carlo_eval(), UINT32_MAX);
        const int end_status = end_game_status();
        if (end_status != 0)
            return make_pair(end_status, UINT32_MAX);
        board.list_empty_pos();
        vector<unsigned int> empty_pos = *board.get_empty_pos();

        const unsigned d1 = depth - 1;
        unordered_map<string, unsigned int> &memo = memos[d1];

        int val = INT_MAX;
        unsigned int best_move = UINT32_MAX;
        for (unsigned int pos : empty_pos) // loop invariant: board is identical as if unchanged
        {
            // memoize search result of childs
            board.set(pos, blk);
            auto [iter, tb_miss] = memo.try_emplace(board.to_string());
            int val_next;
            if (tb_miss)
            {
                val_next = max_search(a, b, d1).first;
                iter->second = val_next;
            }
            else
                val_next = iter->second;
            board.set(pos, non);
            // ab prune
            if (val_next < val)
            {
                val = val_next;
                best_move = pos;
                b = min(b, val);
            }
            if (val <= a)
                break;
        }
        return make_pair(val, best_move);
    }
};

/** Accepts moves and indicates that a game ends*/
class hex_game_ui
{
public:
    hex_graph board;
    const hex_status player_side;
    const hex_status engine_side;
    const bool engine_is_white;
    /**
     * @param is input stream.
     * @param os output stream.
     * @param len board side length.
     * @param is_white_side player is playing white.
     * @param depth Search depth. Must be >0.
     * @param n_trials Number of trials in each Monte-Carlo simulation.
     * @param n_threads Number of threads during each board evaluation.
     */
    hex_game_ui(istream &is, ostream &os, unsigned int len, bool is_white_side, unsigned int depth,
                unsigned int n_trials, unsigned int n_threads) : is(is),
                                                                 os(os),
                                                                 board(len),
                                                                 player_side(is_white_side ? wht : blk),
                                                                 engine_side(is_white_side ? blk : wht),
                                                                 engine_is_white(!is_white_side),
                                                                 engine(board, depth, n_trials, n_threads),
                                                                 end(false)
    {
        if (!is_white_side)
        { // play middle if engine is white
            const unsigned int mid_idx = board.len / 2;
            board.set(mid_idx, mid_idx, engine_side);
        }
        os << "Hello. Do you want to play with me?\n"
           << board << endl;
    }
    /** Accept a move from user via input stream, show the update board, let the engine compute a move, and update the board again.*/
    void make_move()
    {
        unsigned int row;
        unsigned int col;
        os << "Your turn. White (O) goes bottom to top, Black (X) goes left to right. ";
        while (true)
        {
            os << "Enter row (top to down, 1 to " << board.len << ") and col (left to right, 1 to " << board.len << ") of your move:" << endl;
            is >> row >> col;

            if (is.fail() || row <= 0 || row > board.len || col <= 0 || col > board.len)
            {
                is.clear();
                is.ignore(numeric_limits<streamsize>::max(), '\n'); // ignore the rest of the user input line
                os << "Input must be two non-negative integers from 1 to " << board.len << ". Please try again.\n";
                continue;
            }
            row--;
            col--;
            if (board.is_legal_move(row, col))
                break;
            is.clear();
            is.ignore(numeric_limits<streamsize>::max(), '\n'); // ignore the rest of the user input line
            os << "That position is already occupied. Please try again.\n";
        }
        board.set(row, col, player_side);
        if (display_and_test_win())
        {
            os << "YOU WIN!" << endl;
            return;
        }
        os << "Engine's turn..." << flush;
        auto start_time = chrono::high_resolution_clock::now();
        board.set(engine.alpha_beta_search(engine_is_white), engine_side);
        auto stop_time = chrono::high_resolution_clock::now();
        os << " Time taken: " << chrono::duration<double>(stop_time - start_time).count() << 's' << endl;
        if (display_and_test_win())
            os << "YOU LOSE!" << endl;
    }
    inline bool game_over()
    {
        return end;
    }

private:
    istream &is;
    ostream &os;
    game_engine engine;
    /** Whether game has ended*/
    bool end;
    inline bool display_and_test_win()
    {
        os << board << endl;
        end = board.game_status() != non;
        return end;
    }
};

int main()
{
    // Feel free to adjust these parameters if your machine has different perfornce then mine.
    // Game parameters
    const unsigned int board_len = 11;
    const unsigned int play_white = true;
    // Engine parameters to adjust thinking time
    const unsigned int engine_search_depth = 2;  // exponential to time
    const unsigned int n_trial_per_board = 1024; // linear to time
    const unsigned int n_thread = 4;
    hex_game_ui g(cin, cout, board_len, play_white, engine_search_depth, n_trial_per_board, n_thread);
    while (!g.game_over())
        g.make_move();
    return 0;
}