using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class H_InterestLocation : MonoBehaviour
{
    public bool IsOccupied { get; private set; }

    // M�todo para marcar la localizaci�n como ocupada
    public void Occupy()
    {
        IsOccupied = true;
    }

    // M�todo para marcar la localizaci�n como libre
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
            HoolBehaviour hooligan = agent.GetComponent<HoolBehaviour>();
            if (hooligan != null && hooligan.currentTarget == transform)
            {
                Debug.Log("Hooligan has reached the interest location.");
                Occupy();
                agent.isStopped = true;
            }
        }
    }
}
