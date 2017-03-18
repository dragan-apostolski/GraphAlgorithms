package Graphs.DirectedMatrixGraph;


import java.util.Arrays;

public class Graph {

    private int numberOfNodes;
    private Double [][] adjacencyMatrix;


    public Graph(int numberOfNodes) {
        this.numberOfNodes = numberOfNodes;
        this.adjacencyMatrix = new Double[numberOfNodes][numberOfNodes];
        for (Double[] anAdjacencyMatrix : adjacencyMatrix) {
            Arrays.fill(anAdjacencyMatrix, Double.POSITIVE_INFINITY);
        }
        for (int i = 0; i < numberOfNodes; i++) {
            adjacencyMatrix[i][i] = 0d;
        }
    }

    public Graph(int numberOfNodes, Double [][] adjacencyMatrix){
        this.numberOfNodes = numberOfNodes;
        this.adjacencyMatrix = adjacencyMatrix;
    }

    public void addEdge(int i, int j, Double weight){
        adjacencyMatrix[i][j] = weight;
    }


    private double weight(int i, int j){
        if(i == j) return 0;
        else if (adjacencyMatrix[i][j] == null) return Double.MAX_VALUE;
        return adjacencyMatrix[i][j];
    }


    /**
     * Implementation of the Floyd - Warshall algorithm for computing all - pair shortest
     * paths in a directed graph with negative edge weights. It is assumed that the graph does
     * not have a negative weight cycle.
     * This method also computes the predecessor matrix P, where P[i][j] represents the predecessor
     * of vertex j on the shortest path from i to j.
     *
     * The running time of this method is O(V^3).
     *
     * @return the matrix of the all-pair shortest paths
     * */
    public Double [][] floydWarshall(){
        int n = adjacencyMatrix.length;
        Double [][] D = adjacencyMatrix;
        Integer [][] P = new Integer[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if(i == j || weight(i, j) == Double.POSITIVE_INFINITY)
                    P[i][j] = null;
                else if (weight(i, j) < Double.POSITIVE_INFINITY)
                    P[i][j] = i;
            }
        }
        for (int k = 0; k < n; k++) {
            Double [][] Dk = new Double[n][n];
            Integer [][] Pk = new Integer[n][n];
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    if(D[i][j] <= D[i][k] + D[k][j]) {
                        Dk[i][j] = D[i][j];
                        Pk[i][j] = P[i][j];
                    }
                    else {
                        Dk[i][j] = D[i][k] + D[k][j];
                        Pk[i][j] = P[k][j];
                    }
                }
            }
            D = Dk;
            P = Pk;
        }
        return D;
    }


    /**
     * A method that computes the transitive closure of the graph, that is it returns a Boolean matrix T,
     * where T[i][j] is true if there is a path from i to j in the graph, false otherwise.
     *
     * Algorithm runs in O(V^3) time.
     *
     * @return the matrix of transitive closure of the graph
     */
    public Boolean [][] transitiveClosure(){
        int n = numberOfNodes;
        Boolean [][] T = new Boolean[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                T[i][j] = (i == j || weight(i, j) < Double.POSITIVE_INFINITY);
            }
        }
        for (int k = 0; k < n; k++) {
            Boolean [][] Tk = new Boolean[n][n];
            for (int i = 0; i < n; i++) {
                for (int j = 0; j < n; j++) {
                    Tk[i][j] = (T[i][j] || (T[i][k] && T[k][j]));
                }
            }
            T = Tk;
        }
        return T;
    }

    /**
     * Function for computing shortest paths i -> j (for all (i, j) e V) when using at most m edges.
     * The m parameter is specified when iterating and calling this function for each m (1...n-1). This function
     * is invoked by either {@link Graph#fasterShortestPaths} or {@link Graph#slowShortestPaths} functions.
     *
     * @param L^(m-1) the length of the shortest path from i -> j using <= m -1 edges should be a L[i][j].
     * @return the newly computed shortest path matrix now using <= m edges.
     */
    private Double [][] extendShortestPaths(Double[][] L, Double [][] W){
        int n = numberOfNodes;
        Double [][] Lprime = new Double[n][n];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                Lprime[i][j] = Double.POSITIVE_INFINITY;
                for (int k = 0; k < n; k++)
                    Lprime[i][j] = Math.min(Lprime[i][j], L[i][k] + W[k][j]);
            }
        }
        return Lprime;
    }

    public Double [][] fasterShortestPaths(){
        int n = numberOfNodes;
        Double [][] L = new Double[n][n];
        for (int i = 0; i < adjacencyMatrix.length; i++) {
            for (int j = 0; j < adjacencyMatrix.length; j++) {
                L[i][j] = weight(i, j);
            }
        }
        int m = 1;
        while (m < n){
            L = extendShortestPaths(L, L);
            m = 2*m;
        }
        return L;
    }

    private Double [][] slowShortestPaths(){
        int n = numberOfNodes;
        Double [][] L = adjacencyMatrix;
        for (int i = 0; i < adjacencyMatrix.length; i++) {
            for (int j = 0; j < adjacencyMatrix.length; j++) {
                L[i][j] = adjacencyMatrix[i][j] = weight(i, j);
            }
        }
        for(int m = 2; m < n ; m++){
            L = extendShortestPaths(L, adjacencyMatrix);
        }
        return L;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (Double[] doubles : adjacencyMatrix) {
            sb.append(Arrays.toString(doubles)).append("\n");
        }
        return sb.toString();
    }
}