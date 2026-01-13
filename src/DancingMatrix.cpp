#include "../include/DancingMatrix.h"
#include "DynamicGraph.cpp"

//构造函数
DancingMatrix::DancingMatrix( int rows, int cols, int** matrix, bool verbose )  
    : ROWS(rows), COLS(cols), enableGraphSync(verbose) {
    EXIST_ROWS = rows;  
    ColIndex = std::make_unique<ColumnHeader[]>(cols + 1);  
    RowIndex = std::make_unique<RowNode[]>(rows);  
    // if (verbose) graph = make_unique<ConnectedGraph>(rows, cols);

    root = &ColIndex[0];  
    ColIndex[0].left = &ColIndex[COLS];  
    ColIndex[0].right = &ColIndex[1];  
    ColIndex[COLS].right = &ColIndex[0];  
    ColIndex[COLS].left = &ColIndex[COLS-1];  

    for( int i = 1; i < cols; i++ )  
    {  
        ColIndex[i].left = &ColIndex[i-1];  
        ColIndex[i].right = &ColIndex[i+1];  
    }  

    for ( int i = 0; i <= cols; i++ )  
    {  
        ColIndex[i].up = &ColIndex[i];  
        ColIndex[i].down = &ColIndex[i];  
        ColIndex[i].col = i;  
    }  
    ColIndex[0].down = &RowIndex[0];  
    
    dataNodes.reserve(rows * cols / 2);

    // graph = std::make_shared<ConnectedGraph>(ROWS, COLS);
    for( int i = 0; i < rows; i++ ){
        for( int j = 0; j < cols ; j++ ) {  
            if(matrix[i][j] == 1){
                insert(  i , j+1 );  //行数与原矩阵相同，而列数加1
                rowsSet.insert(i);
                colsSet.insert(j+1); 
            }
        }
    }

    InitBlock = Block(rowsSet, colsSet);
    // if(verbose) graph = make_unique<ConnectedGraph>(*this);
    std::cout<< "初始化舞蹈链完成." << endl;
}

// 从文件构造舞蹈链矩阵
DancingMatrix::DancingMatrix( const string& file_path, int from, bool useIg , bool useETT  ) 
    : useIG(useIg), useETT(useETT)
{
    ifstream file(file_path);
    if (!file.is_open()) {
        cerr << "无法打开文件: " << file_path << endl;
        throw runtime_error("无法打开文件");
    }

    string line;
    getline(file, line);  // 读取第一行 

    int rows, cols;
    if( from == 1 ) {
        PreProccess::extractNM( line, cols, rows );
        getline(file, line); 
    } else {
        istringstream iss(line);
        iss >> cols >> rows;
    }

    ROWS = rows;
    COLS = cols;

    if (ROWS > MAX_ROW) {
        cerr << "矩阵行数过大，无法处理: " << ROWS << " 行." << endl;
        throw runtime_error("time out");
    }

    // cout << "处理矩阵维度: " << rows << " 行, " << cols << " 列." << endl;
    ColIndex = std::make_unique<ColumnHeader[]>(cols + 1);  
    RowIndex = std::make_unique<RowNode[]>(rows);  
    root = &ColIndex[0];  

    ColIndex[0].left = &ColIndex[COLS];  
    ColIndex[0].right = &ColIndex[1];  
    ColIndex[COLS].right = &ColIndex[0];  
    ColIndex[COLS].left = &ColIndex[COLS-1];  

    for( int i = 1; i < cols; i++ )  
    {  
        ColIndex[i].left = &ColIndex[i-1];  
        ColIndex[i].right = &ColIndex[i+1];  
    }  

    for ( int i = 0; i <= cols; i++ )  
    {  
        ColIndex[i].up = &ColIndex[i];  
        ColIndex[i].down = &ColIndex[i];  
        ColIndex[i].col = i;  
    }  
    ColIndex[0].down = &RowIndex[0]; 

    // if(useETT) {
    //     DynamicHypergraphCC::Config config;
    //     config.enable_parallel = true;
    //     config.enable_persistence = true;
    //     config.num_threads = 4;
    //     dynamic_hypergraph_cc = make_unique<DynamicHypergraphCC>(config);
    // }

    dataNodes.reserve(rows * cols / 2);
    int currentRow = 0;
    while (getline(file, line)) {
        if (line.empty()) continue; // 跳过空行
        istringstream iss(line);

        string token;
        if (from == 1 || from == 3) {
            iss >> token;
        } else if (from == 2) {
            iss >> token;
            iss >> token;
        }

        int currentCol; 
        // std::vector<int> rowCols;
        while(iss >> currentCol) {
            if (currentCol < 1 || currentCol > cols) {
                cerr << "无效的列索引: " << currentCol << " 在行 " << currentRow + 1 << endl;
                exit(1);
            }
            insert(currentRow, currentCol); // 插入节点
            col_to_rows[currentCol].push_back(currentRow);
            row_to_cols[currentRow].insert(currentCol);
            ONE_COUNT++; // 统计矩阵中1的个数
            rowsSet.insert(currentRow);
            colsSet.insert(currentCol); 
            // rowCols.push_back(currentCol);
        }

        // 初始化ETT节点
        // if (useETT) detector->add_row(currentRow);
        // active_rows.insert(currentRow);
        // if (useETT) {
        //     dynamic_hypergraph_cc->addRow(currentRow, rowCols);
        // }
        currentRow++;
        
        if (currentRow >= rows) break; // 防止超过预期行数
    }

    InitBlock = Block(rowsSet, colsSet);

    if(useETT){
        detector = make_unique<ComponentDetector>(ROWS, COLS); 
        detector->Initialize(col_to_rows);

        cout << "ETT initialization complete." << endl;
    }

    if (useIg) {
        incrementalGraph = make_unique<IncrementalConnectedGraph>(rows);
        incrementalGraph->initialize(*this);
    }

    file.close();
}

DancingMatrix::~DancingMatrix() = default;

vector<Block> DancingMatrix::getComponentsByIG(const set<int> rows) {
    return incrementalGraph->computeComponentsInRows(rows);
    // return findComponents(rows);
};


//插入元素到双向十字链表中
void DancingMatrix::insert( int r, int c )  
{  
    ColIndex[c].size++;  
    RowIndex[r].size++;
    // ColIndex[c].rows.insert(r);
    // RowIndex[r].cols.insert(c);
    auto newNode = std::make_unique<Node>(r, c);  
    Node* newNodePtr = newNode.get();

    Node* cur = &ColIndex[c];  
    while( cur->down != &ColIndex[c] && cur->down->row < r )  
        cur = cur->down;  

    newNodePtr->down = cur->down;  
    newNodePtr->up = cur;  
    cur->down->up = newNodePtr;  
    cur->down = newNodePtr;  
    if( RowIndex[r].right == nullptr )  
    {  
        RowIndex[r].right = newNodePtr; 
        newNodePtr->row_first_node = true;
        newNode->left = newNodePtr;  
        newNode->right = newNodePtr;  
    }  
    else  
    {  
        Node* rowHead = RowIndex[r].right;  
        cur = rowHead;  

        while( cur->right != rowHead && cur->right->col < c )  
            cur = cur->right;  

        newNodePtr->right = cur->right;  
        newNodePtr->left = cur;  
        cur->right->left = newNodePtr;  
        cur->right = newNodePtr;  
    }  

    dataNodes.push_back(std::move(newNode));
}

string DancingMatrix::encodeBlockState(const unordered_set<int>& cols){
    string state(COLS, '0'); // 初始化为全0字符串
    for(int col : cols) {
        state[col - 1] = '1'; // 将对应列设置为1   
    }
    return state;
}

size_t DancingMatrix::hashBlockState(const set<int>& cols) {
    size_t hash = 0;
    for(int col : cols) {
        hash ^= std::hash<int>()(col) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
    }
    return hash;
}

//获取当前列的状态
size_t DancingMatrix::getColumnState() const {
    size_t hash = 0;
    ColumnHeader* cur = (ColumnHeader*)root->right;
    while (cur != root) {
        hash ^= std::hash<int>()(cur->col) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
        cur = (ColumnHeader*)cur->right;
    }
    return hash;
}

// 创建当前状态的签名
Signature DancingMatrix::getColumnSignature() const {
    Signature sig;
    sig.covered.resize(COLS + 1, false);
    
    // 标记已覆盖的列
    for (int j = 1; j <= COLS; j++) {
        bool found = false;
        for (int k = root->right->col; k != 0; k = ColIndex[k].right->col) {
            if (k == j) {
                found = true;
                break;
            }
        }
        sig.covered[j] = !found;
    }
    
    return sig;
}

void DancingMatrix::build_mapping_from_cols(const unordered_set<int>& blockCols, unordered_map<int, set<int>>& rowToCols, unordered_map<int, set<int>>& colToRows)
{
    for (auto col : blockCols) {
        ColumnHeader* c = &ColIndex[col];
        Node* curR = c->down;
        while (curR != c) {
            rowToCols[curR->row].insert(col);
            colToRows[col].insert(curR->row);
            curR = curR->down;
        }
    }
}

void DancingMatrix::printMatrix() const
{
    std::cout<< "Remain Matrix Nodes: " << std::endl;
    ColumnHeader* current = (ColumnHeader*)root->right;
    while(current != root)
    {
        std::cout << "Column " << current->col << " size: " << current->size << " ";
        if(current->size > 0){
            Node* cur = current->down;
            std::cout << "{ Rows: ( ";
            while(cur != current)
            {
                std::cout << cur->row + 1;
                cur = cur->down;
                if(cur != current)
                    std::cout << ", ";
            }
            std::cout << " ) } " << std::endl;
        }
        current = (ColumnHeader*)current->right;
    }
    std::cout << std::endl;
}

void DancingMatrix::printBlocks( vector<Block>& blocks) const {
    int i = 1;
    for (auto& block : blocks) {
        block.printBlock(i++);
    }
}

void DancingMatrix::cover( int c )  
{  
    ColumnHeader* col = &ColIndex[c];  
    col->right->left = col->left;  
    col->left->right = col->right; 
    colsSet.erase(c);

    Node* curR, *curC;  
    curC = col->down;  
    while( curC != col )  
    {   
        Node* noteR = curC;  

        curR = noteR->right;  
        while( curR != noteR )  
        {  
            curR->down->up = curR->up;  
            curR->up->down = curR->down;  
            --ColIndex[curR->col].size;  

            curR = curR->right;  
        }  
        rowsSet.erase(curC->row);
        curC = curC->down;  
    }  
}

void DancingMatrix::uncover( int c )  
{  
    Node* curR, *curC;  
    ColumnHeader* col = &ColIndex[c];  
    curC = col->up;  
    while( curC != col )  
    {  
        Node* noteR = curC;  

        curR = curC->left;  
        while( curR != noteR )  
        {  
            ++ColIndex[curR->col].size;  
            curR->down->up = curR;  
            curR->up->down = curR;  

            curR = curR->left;  
        }  
        rowsSet.insert(curC->row);
        curC = curC->up;  
    }  
    col->right->left = col;  
    col->left->right = col;  
    colsSet.insert(c);
}

void DancingMatrix::coverInBlock(int c, Block& block, set<int>& removed_rows){

    ColumnHeader* col = &ColIndex[c];  
    col->right->left = col->left;  
    col->left->right = col->right;  
    
    block.cols.erase(c); // 从块中移除列

    Node* curR, *curC;  
    curC = col->down;  

    while( curC != col )  
    {    
        int row_id = curC->row;

        if (isGraphSyncEnabled() && useIG) {
            incrementalGraph->deactivateRow(row_id);
        }
        block.rows.erase(row_id); // 从块中移除行

        curR = curC->right;  
        while( curR != curC )  
        {          
            curR->down->up = curR->up;  
            curR->up->down = curR->down;  
            --ColIndex[curR->col].size;
            curR = curR->right;  
        }  

        curC = curC->down;  
    } 
}

void DancingMatrix::uncoverInBlock(int c, Block& block){ 
    ColumnHeader* col = &ColIndex[c];  
    Node* curC = col->up; 

    while( curC != col )  
    {  
        int row_id = curC->row;
 
        Node* curR = curC->left;  
        while( curR != curC )  
        {  
            ++ColIndex[curR->col].size;
            curR->down->up = curR;  
            curR->up->down = curR;  
            curR = curR->left;  
        }  

        block.rows.insert(row_id); // 将行添加到块中

        if (isGraphSyncEnabled() && useIG) {
            incrementalGraph->reactivateRow(row_id); 
        }

        curC = curC->up;  
    }  

    col->right->left = col;  
    col->left->right = col;  
    block.cols.insert(c);
}

ColumnHeader* DancingMatrix::selectColumnHeuristic(const set<int>& cols) {
    ColumnHeader* chosen = nullptr;
    int minSize = INT_MAX;

    for (int col : cols) {
        if (col < 1 || col > COLS) continue; // 跳过无效列
        int sz = getColSize(col);

        if (sz < minSize) {
            minSize = sz;
            chosen = getColumnHeader(col);
        }
    }
    return chosen;
}

ColumnHeader* DancingMatrix::selectColumnByLinear(const set<int>& cols, int threshold) {
    
    ColumnHeader* bestCol = getColumnHeader(*cols.begin()); // 保底选择第一列
    int bestSize = getColSize(bestCol->col);
    
    for (int colId : cols) {
        int size = getColSize(colId);
        
        if (size < bestSize) {
            bestCol = getColumnHeader(colId);
            bestSize = size;
        }
    }
    
    return bestCol; // 保证非空返回
}

ColumnHeader* DancingMatrix::selectColumnByMinHeap(const set<int>& cols, int threshold) {
    
    // 使用小根堆选择最小列
    priority_queue<ColumnInfo, vector<ColumnInfo>, greater<ColumnInfo>> minHeap;
    
    for (int colId : cols) {
        int size = getColSize(colId);
        minHeap.push({colId, size, getColumnHeader(colId)});
    }
    
    // 保底返回：记录第一个（最小size）列
    ColumnInfo firstCol = minHeap.top();
    ColumnHeader* bestCol = firstCol.header;
    int bestDistance = abs(firstCol.size - threshold);
    
    // 从小到大取出元素，寻找最接近阈值的列
    while (!minHeap.empty()) {
        ColumnInfo current = minHeap.top();
        minHeap.pop();
        
        int distance = abs(current.size - threshold);
        
        // 更新最优列
        if (distance < bestDistance) {
            bestDistance = distance;
            bestCol = current.header;
        }
        
        // 早期退出优化：
        // 1. 找到精确匹配
        if (distance == 0) break;
        
        // 2. 当前size已经超过阈值，且距离开始增大时停止
        //    因为后续元素size更大，距离只会越来越大
        if (current.size > threshold && distance > bestDistance) {
            break;
        }
    }
    
    return bestCol;  // 保证非空
}

ColumnHeader* DancingMatrix::selectOptimalColumn(const set<int>& cols) {
    
    // if (cols.size() <= HEAP_THRESHOLD) {
    //     return selectColumnByLinear(cols, TARGET_THRESHOLD);
    // }
    if (single_thread_mode) {
        return selectColumnByMinHeap(cols, TARGET_THRESHOLD);
    }

    return selectColumnByLinear(cols, TARGET_THRESHOLD);
}

ColumnHeader* DancingMatrix::selectCol()
{
    ColumnHeader* choose = getColumnHeader(root->right->col), *cur=choose;  
    while( cur != root )  
    {   //选择元素最少的列
        int chooseSize = getColSize(choose->col);
        int curSize = getColSize(cur->col);
        if( chooseSize > curSize )  
            choose = cur;  
        cur = getColumnHeader(cur->right->col);  
    } 
    return choose;
}

col_id DancingMatrix::getClosedSizeCol(const int expected_size) {
    ColumnHeader* choose = (ColumnHeader*)root->right, *cur=choose;  
    while( cur != root )  
    {   //选择接近预期大小的列
        if( abs(choose->size - expected_size) > abs(cur->size - expected_size) )  
            choose = cur;  
        cur = (ColumnHeader*)cur->right;  
    } 
    return choose->col;
}

col_id DancingMatrix::getSmallestSizeCol() {
    ColumnHeader* choose = (ColumnHeader*)root->right, *cur=choose;  
    while( cur != root )  
    {   //选择元素最少的列
        if( choose->size > cur->size )  
            choose = cur;  
        cur = (ColumnHeader*)cur->right;  
    } 
    return choose->col;
}

void PreProccess::extractNM(const std::string& line, int& n, int& m) {
    std::istringstream iss(line);
    std::string token;

    // 读取 "c"
    iss >> token;  // 读取 "c"

    iss >> token;  // 读取 "n"
    if (token == "n") {
        iss >> token;  // 读取 "="
        iss >> n;      // 读取 n 的值
    } else {
        throw std::runtime_error("格式错误: 未找到 'n'");
    }

    // 读取 "m" 和其值
    iss >> token;  // 读取 ","
    iss >> token;  // 读取 "m"
    if (token == "m") {
        iss >> token;  // 读取 "="
        iss >> m;      // 读取 m 的值
    } else {
        throw std::runtime_error("格式错误: 未找到 'm'");
    }
}

int** PreProccess::processFileToMatrix1(const std::string& filename, int& r, int& c) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件");
    }

    std::string line;
    std::getline(file, line);  // 读取第一行

    int n, m;
    extractNM(line, n, m);  // 读取第一行的 n 和 m

    r = m;
    c = n;

    std::getline(file, line);  // 跳过第二行(第二个数据集不用跳过)

    // 动态分配二维数组
    int** matrix = new int*[r];
    for (int i = 0; i < r; ++i) {
        matrix[i] = new int[c];
        for (int j = 0; j < c; ++j) {
            matrix[i][j] = 0;  // 初始化为0
        }
    }

    int row = 0;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] != 's') continue;  // 忽略不是以 's' 开头的行
        std::istringstream iss(line.substr(2));  // 跳过前两个字符's '
        //std::istringstream iss(line);
        int col;
        while (iss >> col) {
            if (col > 0 && col <= n) {  // 确保列索引在有效范围内
                matrix[row][col - 1] = 1;  // 由于数组索引从0开始，所以减1
            }
        }
        ++row;
    }

    file.close();
    return matrix;
}

int** PreProccess::processFileToMatrix2(const fs::path& filename, int& r, int& c) {
    std::ifstream file(filename.string());
    if (!file.is_open()) {
        throw std::runtime_error("无法打开文件");
    }

    std::string line;
    std::getline(file, line);  // 读取第一行

    int n, m;
    std::istringstream iss(line);
    iss >> n >> m;  // 假设第一行格式为 "n m"  
    c = n;
    r = m;

    // 动态分配二维数组
    vector<vector<int>> tmp_matrix;

    while (std::getline(file, line)) {
        vector<int> tmp_row(c, 0);  // 初始化当前行
        std::istringstream iss(line);
        int tmp;
        iss >> tmp;   // 跳过开头的数字
        iss >> tmp;  // 读取该行“1”的个数

        int col;
        for (int i = 0; i < tmp; ++i) {
            iss >> col;  // 读取列索引
            if (col > 0 && col <= n) {  // 确保列索引在有效范围内
                tmp_row[col - 1] = 1;  // 由于数组索引从0开始，所以减1
            }
        }
        tmp_matrix.push_back(tmp_row);
    }
    file.close();
    
    r = tmp_matrix.size();
    // 动态分配二维数组
    int** matrix = new int*[r];
    for(int i = 0; i < r; ++i) {
        matrix[i] = new int[c];
        for(int j = 0; j < c; ++j) {
            matrix[i][j] = tmp_matrix[i][j];  // 将1填入矩阵
        }
    }
    return matrix;
}

int** PreProccess::processFileToMatrix3(const std::string& filename, int& r, int& c) { 
    std::ifstream file(filename);
    if(!file.is_open()) {
        throw std::runtime_error("无法打开文件");
    }

    std::string line;
    std::getline(file, line);  // 读取第一行
    std::istringstream iss(line);
    int n, m;
    iss >> n >> m;  // 假设第一行格式为 "n m"
    c = n;
    r = m;

    // 动态分配二维数组
    int** matrix = new int*[r];
    for(int i = 0; i < r; ++i) {
        matrix[i] = new int[c];
        for(int j = 0; j < c; ++j) {
            matrix[i][j] = 0;
        }
    }

    int row = 0;
    while (std::getline(file, line))
    {
        std::istringstream rowStream(line);
        int col, count;

        // 读取该行“1”的个数
        rowStream >> count;

        for (int i = 0; i < count; ++i) {
            rowStream >> col;  // 读取列索引
            if (col > 0 && col <= n) {  // 确保列索引在有效范围内
                matrix[row][col - 1] = 1;  // 由于数组索引从0开始，所以减1
            }
        }
        row++;
    }
    
    file.close();
    return matrix;
}

void PreProccess::freeMatrix(int** matrix, int rows) {
    for (int i = 0; i < rows; ++i) {
        delete[] matrix[i];
    }
    delete[] matrix;
}