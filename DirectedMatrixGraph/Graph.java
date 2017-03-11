package Graphs.DirectedMatrixGraph;

import java.util.Arrays;
import java.util.Scanner;

public class Graph {

    private int numberOfNodes;
    private Double [][] adjacencyMatrix;
    private int [][] predecessorMatrix;


    public Graph(int numberOfNodes) {
        this.numberOfNodes = numberOfNodes;
        this.adjacencyMatrix = new Double[numberOfNodes][numberOfNodes];
        this.predecessorMatrix = new int[numberOfNodes][numberOfNodes];
        for (int i = 0; i < adjacencyMatrix.length; i++) {
            adjacencyMatrix[i][i] = 0d;
        }
    }

    public Graph(int numberOfNodes, Double [][] adjacencyMatrix){
        this.numberOfNodes = numberOfNodes;
        this.adjacencyMatrix = adjacencyMatrix;
        this.predecessorMatrix = new int[numberOfNodes][numberOfNodes];
    }

    public void addEdge(int i, int j, Double weight){
        adjacencyMatrix[i][j] = weight;
    }

    private double weight(int i, int j){
        if(i == j) return 0;
        else if (adjacencyMatrix[i][j] == null) return Double.MAX_VALUE;
        return adjacencyMatrix[i][j];
    }


    /**Function for computing shortest paths i -> j (for all (i, j) e V) when using at most m edges.
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

    public static void main(String[] args) {
        Graph g = new Graph(5);
        Scanner sc = new Scanner(System.in);
        for (int i = 0; i < 9; i++) {
            g.addEdge(sc.nextInt(), sc.nextInt(), sc.nextDouble());
        }
        System.out.println(new Graph(5, g.fasterShortestPaths()));
    }
}
