#pragma once
#include <Arduino.h>

// ================= NodeConnections =================
class NodeConnections {
public:
  // -1 = sem conexão
  int16_t connections[4];
  uint8_t connectionsIndex = 0;

  NodeConnections() { for (auto &c : connections) c = -1; }

  void setConnection(int16_t idxNode, int8_t at = -1) {
    if (idxNode < 0) return;
    if (at >= 0 && at < 4) { connections[at] = idxNode; return; }
    if (connectionsIndex < 4) connections[connectionsIndex++] = idxNode;
  }
};

// ================= List (Regra dos 5) =================
class List {
private:
  int16_t*  buf      = nullptr;
  uint16_t  capacity = 0;
  uint16_t  length   = 0;

  void ensureCapacity(uint16_t want) {
    if (want <= capacity) return;
    uint16_t newCap = max<uint16_t>(1, capacity ? capacity * 2 : 4);
    while (newCap < want) newCap *= 2;
    resize(newCap);
  }

public:
  explicit List(uint16_t cap = 0): capacity(cap) {
    if (cap) {
      buf = new int16_t[cap];
      for (uint16_t i=0;i<cap;i++) buf[i] = 0;
    }
  }

  ~List(){ delete[] buf; }

  // copy ctor
  List(const List& other): capacity(other.capacity), length(other.length) {
    if (capacity) {
      buf = new int16_t[capacity];
      for (uint16_t i=0;i<length;i++) buf[i] = other.buf[i];
    }
  }

  // copy assign
  List& operator=(const List& other){
    if (this != &other) {
      int16_t* nb = (other.capacity ? new int16_t[other.capacity] : nullptr);
      for (uint16_t i=0;i<other.length;i++) nb[i] = other.buf[i];
      delete[] buf;
      buf = nb;
      capacity = other.capacity;
      length   = other.length;
    }
    return *this;
  }

  // move ctor
  List(List&& other) noexcept : buf(other.buf), length(other.length), capacity(other.capacity) {
    other.buf=nullptr; other.length=0; other.capacity=0;
  }

  // move assign
  List& operator=(List&& other) noexcept {
    if (this != &other) {
      delete[] buf;
      buf=other.buf; length=other.length; capacity=other.capacity;
      other.buf=nullptr; other.length=0; other.capacity=0;
    }
    return *this;
  }

  void resize(uint16_t newCap) {
    int16_t* old = buf;
    uint16_t oldLen = length;
    buf = (newCap ? new int16_t[newCap] : nullptr);
    capacity = newCap;
    length = min(oldLen, newCap);
    for (uint16_t i=0;i<length;i++) buf[i] = old ? old[i] : 0;
    if (old) delete[] old;
  }

  void append(int16_t v) {
    ensureCapacity(length + 1);
    buf[length++] = v;
  }

  void setByIndex(uint16_t i, int16_t v){ if (i < length) buf[i] = v; }

  void removeByIndex(uint16_t i){
    if (i >= length) return;
    for (uint16_t j=i+1;j<length;j++) buf[j-1] = buf[j];
    length--;
  }

  int16_t getByIndex(uint16_t i) const { return (i < length) ? buf[i] : (int16_t)-32768; } // sentinel

  uint16_t len() const { return length; }

  bool contains(int16_t v) const { for (uint16_t i=0;i<length;i++) if (buf[i]==v) return true; return false; }

  void clear(){ length = 0; }

  void print() const {
    // Opcional: implemente se for depurar via Serial
  }
};

// ================= Point =================
struct Point {
  int16_t row = 0;
  int16_t col = 0;
  Point(int16_t r=0, int16_t c=0): row(r), col(c) {}
};

// ================= MazeRunner =================
class MazeRunner {
private:
  uint16_t        n_nodes = 0;
  uint8_t         nrows = 0, ncols = 0;
  uint8_t         robotDirection = 0; // 0:up 1:right 2:down 3:left
  Point           actualPoint = Point();
  NodeConnections* nodeConnections = nullptr;
  List            visitNodes;   // índices visitados (FIFO do destino)
  int8_t*         visitTimes = nullptr;

  void initNodeConnections(uint16_t N) { nodeConnections = new NodeConnections[N]; }

  bool validIndex(int16_t idx) const { return (idx >= 0) && ((uint16_t)idx < n_nodes); }

public:
  // listas auxiliares públicas (mantidas)
  List blackNodes  = List(10);
  List victimNodes = List(15);
  List blueNodes   = List(10);

  MazeRunner(uint8_t rows, uint8_t cols, Point& start)
    : nrows(rows), ncols(cols), actualPoint(start), visitNodes(16) {
    n_nodes = (uint16_t)nrows * (uint16_t)ncols;
    visitNodes.resize(max<uint16_t>(visitNodes.len(), n_nodes));
    int16_t startIdx = pointToIndex(start);
    if (validIndex(startIdx)) visitNodes.append(startIdx);

    initNodeConnections(n_nodes);
    visitTimes = new int8_t[n_nodes];
    for (uint16_t i=0;i<n_nodes;i++) visitTimes[i]=0;
  }

  ~MazeRunner(){ delete[] visitTimes; delete[] nodeConnections; }

  // ---------- métricas de visita ----------
  void addVisitNodesTimes(int16_t index) { if (validIndex(index)) visitTimes[index]++; }

  int8_t getVisitNodesTimes(uint16_t index) { return (index < n_nodes) ? visitTimes[index] : 0; }

  // ---------- estado do robô ----------
  Point getActualPoint() { return actualPoint; }

  void setRobotDirection(uint8_t newDirection) {
    if (newDirection < 4) robotDirection = newDirection;
  }

  void setActualPoint(Point newPoint) { actualPoint = newPoint; }

  void delVisitNodesByIndex(uint16_t index) { visitNodes.removeByIndex(index); }

  // ---------- helpers ----------
  static List copyList(const List& list) {
    List myList(list.len());
    for (uint16_t i = 0; i < list.len(); i++) myList.append(list.getByIndex(i));
    return myList;
  }

  int16_t pointToIndex(const Point& p) const {
    if (p.row < 0 || p.col < 0 || p.row >= nrows || p.col >= ncols) return -1;
    return (int16_t)(p.row * ncols + p.col);
  }

  Point indexToPoint(uint16_t idx) const { return Point(idx / ncols, idx % ncols); }

  List adjacency(uint16_t idx) {
    List adj(4);
    if (idx >= n_nodes) return adj;
    for (uint8_t j=0;j<4;j++){
      int16_t nb = nodeConnections[idx].connections[j];
      if (validIndex(nb)) adj.append(nb);
    }
    return adj;
  }

  List bfs(uint16_t index) {
    List queue(n_nodes);
    List father(n_nodes);

    // vetor de cor (0 = descoberto/visitado, 1 = não visitado)
    uint8_t* color = new uint8_t[n_nodes];
    for (uint16_t i = 0; i < n_nodes; i++) {
      father.append(-1);
      color[i] = 1;
    }

    if (index >= n_nodes) {
      delete[] color;
      return father;
    }

    color[index] = 0;
    father.setByIndex(index, -1);
    queue.append(index);

    while (queue.len() > 0) {
      int16_t u = queue.getByIndex(0);
      queue.removeByIndex(0);

      List adjacencyList = adjacency(u);
      for (uint16_t i = 0; i < adjacencyList.len(); i++) {
        int16_t j = adjacencyList.getByIndex(i);
        if (j >= 0 && (uint16_t)j < n_nodes && color[j] == 1) {
          color[j] = 0;
          father.setByIndex(j, u);
          queue.append(j);
        }
      }
    }

    delete[] color;
    return father; // cópia/move segura
  }

  List minimumPathBetween(int16_t index_1, int16_t index_2) {
    if (!validIndex(index_1) || !validIndex(index_2)) return List(0);

    List fat = bfs(index_2);
    uint16_t count = 0;
    int16_t auxIndex = index_1;

    // Conta para alocar o path
    while (auxIndex >= 0 && auxIndex != -32768) {
      int16_t f = fat.getByIndex(auxIndex);
      if (f == -32768 || f == -1) break;
      count++;
      auxIndex = f;
    }

    List path(count + 1);
    path.append(index_1);
    while (true) {
      int16_t f = fat.getByIndex(index_1);
      if (f == -32768 || f == -1) break;
      path.append(f);
      index_1 = f;
    }
    return path;
  }

  List getInstructions() {
    if (visitNodes.len()==0) return List(0);
    int16_t dst = visitNodes.getByIndex(0);
    if (!validIndex(dst)) return List(0);

    int16_t srcIdx = pointToIndex(actualPoint);
    if (!validIndex(srcIdx)) return List(0);

    List minimumPath = minimumPathBetween(srcIdx, dst);
    uint16_t len = minimumPath.len();
    if (len < 2) return List(0);

    // Constrói instruções absolutas 0/1/2/3
    List instructions(len - 1);
    Point a, b;
    for (uint16_t i = 0; i+1 < len; i++) {
      a = indexToPoint(minimumPath.getByIndex(i + 1));
      b = indexToPoint(minimumPath.getByIndex(i));
      int8_t dr = a.row - b.row;
      if (dr) instructions.append(dr < 0 ? 0 : 2);
      else {
        int8_t dc = a.col - b.col;
        if (dc) instructions.append(dc < 0 ? 3 : 1);
      }
    }

    // Converte para instruções relativas ao heading atual, invertendo left/right
    List out(instructions.len());
    uint8_t compass = robotDirection;
    for (uint16_t i=0;i<instructions.len();i++){
      out.append( (compass + ((instructions.getByIndex(i)*3)%4)) % 4 );
      compass = instructions.getByIndex(i);
    }
    for (uint16_t i=0;i<out.len();i++){
      if (out.getByIndex(i)==1) out.setByIndex(i,3);
      else if (out.getByIndex(i)==3) out.setByIndex(i,1);
    }
    return out;
  }

  void helpInstructions() {
    // "instructions é um vetor tipo List com a sequência de comandos: 0=up, 1=right, 2=down, 3=left."
  }

  void move(String direction) {
    int16_t index_1 = pointToIndex(actualPoint);

    if (direction == "up") {
      if (actualPoint.row > 0) {
        actualPoint.row--;
        int16_t index_2 = pointToIndex(actualPoint);
        if (validIndex(index_1) && validIndex(index_2)) {
          nodeConnections[index_1].setConnection(index_2);
          nodeConnections[index_2].setConnection(index_1);
        }
        robotDirection = 0;
        if (validIndex(index_2) && !visitNodes.contains(index_2)) visitNodes.append(index_2);
      }
    } else if (direction == "down") {
      if (actualPoint.row < nrows - 1) {
        actualPoint.row++;
        int16_t index_2 = pointToIndex(actualPoint);
        if (validIndex(index_1) && validIndex(index_2)) {
          nodeConnections[index_1].setConnection(index_2);
          nodeConnections[index_2].setConnection(index_1);
        }
        robotDirection = 2;
        if (validIndex(index_2) && !visitNodes.contains(index_2)) visitNodes.append(index_2);
      }
    } else if (direction == "left") {
      if (actualPoint.col > 0) {
        actualPoint.col--;
        int16_t index_2 = pointToIndex(actualPoint);
        if (validIndex(index_1) && validIndex(index_2)) {
          nodeConnections[index_1].setConnection(index_2);
          nodeConnections[index_2].setConnection(index_1);
        }
        robotDirection = 3;
        if (validIndex(index_2) && !visitNodes.contains(index_2)) visitNodes.append(index_2);
      }
    } else if (direction == "right") {
      if (actualPoint.col < ncols - 1) {
        actualPoint.col++;
        int16_t index_2 = pointToIndex(actualPoint);
        if (validIndex(index_1) && validIndex(index_2)) {
          nodeConnections[index_1].setConnection(index_2);
          nodeConnections[index_2].setConnection(index_1);
        }
        robotDirection = 1;
        if (validIndex(index_2) && !visitNodes.contains(index_2)) visitNodes.append(index_2);
      }
    }
  }

  uint8_t getDirection() { return robotDirection; }

  List getVisitNodes() { return visitNodes; } // cópia segura (Regra dos 5)
  
  // direction = 0 (north), 1 (east), 2 (south), 3 (west)
  // return = [front, right, bottom, left] em índices do grid relativo ao heading
  List getRelativeNeighborhood(uint8_t myDirection) {
    List sideIndex(4);
    Point sidePoint[4] = {
      Point(actualPoint.row - 1, actualPoint.col),  // VISUAL TOP NODE
      Point(actualPoint.row,     actualPoint.col + 1),  // RIGHT
      Point(actualPoint.row + 1, actualPoint.col),  // BOTTOM
      Point(actualPoint.row,     actualPoint.col - 1)   // LEFT
    };
    uint8_t vectorAux[4] = {0,1,2,3};
    switch (myDirection) {
      case 0: vectorAux[0]=0; vectorAux[1]=1; vectorAux[2]=2; vectorAux[3]=3; break; // north
      case 1: vectorAux[0]=1; vectorAux[1]=2; vectorAux[2]=3; vectorAux[3]=0; break; // east
      case 2: vectorAux[0]=2; vectorAux[1]=3; vectorAux[2]=0; vectorAux[3]=1; break; // south
      case 3: vectorAux[0]=3; vectorAux[1]=0; vectorAux[2]=1; vectorAux[3]=2; break; // west
      default: break;
    }
    for (uint8_t i = 0; i < 4; i++) sideIndex.append(pointToIndex(sidePoint[vectorAux[i]]));
    return sideIndex;
  }
};
