package Graphs.Network;

import java.util.*;

class ResidualNetwork{

    private final int source;
    private final int sink;
    private final int numberOfVertices;
    private Vertex [] adjList;
    private Network originalNetwork;


    ResidualNetwork(Network network, int source, int sink, int numVertices) {
        this.source = source;
        this.sink = sink;
        this.numberOfVertices = numVertices;
        adjList = new Vertex[numVertices];
        this.originalNetwork = network;
        for (int i = 0; i < numVertices; i++) {
            adjList[i] = new Vertex(i);
        }
        buildResidualNetwork(originalNetwork);
    }

    private void buildResidualNetwork(Network originalNetwork) {
        for (Vertex u : originalNetwork.adjList) {
            for (Vertex v : u.neighbors.keySet()) {
                Edge e = u.getEdge(v);
                addEdge(u.index, v.index, e.capacity);
            }
        }
    }

    private void addEdge(int u, int v, int capacity){
        adjList[u].addNeighbor(adjList[v], capacity);
    }

    private void removeEdge(Vertex u, Vertex v){
        adjList[u.index].removeEdge(v);
    }

    private int getEdgeFlow(int from, int to) {
        return originalNetwork.getEdgeFlow(from, to);
    }

    private boolean edgeExists(int from, int to){
        return originalNetwork.adjList.get(from).hasNeighbor(originalNetwork.adjList.get(to));
    }

    private void updateFlow(int from, int to, int residualCapacity) {
        originalNetwork.updateFlow(from, to, residualCapacity);
        updateResidualNetwork(from, to, residualCapacity);
    }

    private void updateResidualNetwork(int from, int to, int newFlow){
        int originCapacity = originalNetwork.adjList.get(from).getEdge(originalNetwork.adjList.get(to)).capacity;
        adjList[from].addNeighbor(adjList[to], originCapacity - newFlow);
        adjList[to].addNeighbor(adjList[from],  newFlow);
        if(newFlow == originCapacity)
            removeEdge(adjList[from], adjList[to]);
        if(newFlow == 0){
            removeEdge(adjList[to], adjList[from]);
        }
    }


    void fordFulkerson(){
        while(true){
            try {
                List<Edge> path = bfs();
                int residualCapacity = path.stream().mapToInt(Edge::getCapacity).min().orElseThrow(RuntimeException::new);
                for (Edge edge : path) {
                    if(edgeExists(edge.from, edge.to)){
                        updateFlow(edge.from, edge.to, getEdgeFlow(edge.from, edge.to) +  residualCapacity);
                    }
                    else{
                        updateFlow(edge.to, edge.from, getEdgeFlow(edge.to, edge.from) - residualCapacity);
                    }
                }
            }
            catch (NoPathFoundException e){
                break;
            }
        }
    }

    private List<Edge> bfs() throws NoPathFoundException {
        Double [] distance = new Double[numberOfVertices];
        Integer [] predecessor = new Integer[numberOfVertices];
        Arrays.fill(predecessor, -1);
        Arrays.fill(distance, Double.POSITIVE_INFINITY);
        Set<Vertex> visited = new HashSet<>();
        distance[source] = 0d;
        predecessor[source] = source;
        Queue<Vertex> q = new LinkedList<>();
        q.add(adjList[source]);

        while(!q.isEmpty()){
            Vertex u = q.remove();
            visited.add(u);
            for (Vertex v : u.neighbors.keySet()) {
                if(!visited.contains(v)){
                    distance[v.index] = distance[u.index] + 1;
                    q.add(v);
                    predecessor[v.index] = u.index;
                }
            }
        }

        if(distance[sink] == Double.POSITIVE_INFINITY)
            throw new NoPathFoundException();
        return buildPath(predecessor);
    }

    private List<Edge> buildPath(Integer[] predecessor) {
        List<Edge> edges = new ArrayList<>();
        int tmp = sink;
        while(tmp != source){
            int from = predecessor[tmp];
            int to = tmp;
            int residualCapacity = adjList[from].getCapacity(adjList[to]);
            edges.add(0, new Edge(from, to, residualCapacity));
            tmp = predecessor[tmp];
        }
        return edges;
    }
}