package Graphs.Network;

import java.util.Scanner;

public class NetworkFlowTester {

    public static void main(String[] args) {
        Scanner sc = new Scanner(System.in);
        int n = sc.nextInt();
        int m = sc.nextInt();
        Network g = new Network(n, 0, n - 1);
        for (int i = 0; i < m; i++) {
            g.addEdge(sc.nextInt(), sc.nextInt(), sc.nextInt());
        }
        System.out.println(g.maxFlow());
    }
}
