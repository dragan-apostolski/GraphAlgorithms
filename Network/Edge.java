package Graphs.Network;

public class Edge {
    int from, to;
    int capacity;
    private int flow;

    Edge(int from, int to, int capacity) {
        this.capacity = capacity;
        flow = 0;
        this.from = from;
        this.to = to;
    }

    int getCapacity() {
        return capacity;
    }

    void updateFlow(int newFlow){
        flow = newFlow;
    }

    int getFlow() {
        return flow;
    }

    @Override
    public String toString() {
        return "From: " + from + " To: " +  to + " " + flow + " / " + capacity;
    }
}