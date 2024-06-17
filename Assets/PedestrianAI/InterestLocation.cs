using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class InterestLocation : MonoBehaviour
{
    public bool IsOccupied { get; private set; }

    // Método para marcar la localización como ocupada
    public void Occupy()
    {
        IsOccupied = true;
    }

    // Método para marcar la localización como libre
    public void Vacate()
    {
        IsOccupied = false;
    }

    void OnTriggerEnter(Collider other)
    {
        NavMeshAgent agent = other.GetComponent<NavMeshAgent>();
        if (agent != null)
        {
            // Optionally check if the NavMeshAgent has a specific script or tag
            PedestrianBehaviour pedestrian = agent.GetComponent<PedestrianBehaviour>();
            if (pedestrian != null && pedestrian.currentTarget == transform)
            {
                Debug.Log("Pedestrian has reached the interest location.");
                Occupy();
                agent.isStopped = true;
            }
        }
    }
}
