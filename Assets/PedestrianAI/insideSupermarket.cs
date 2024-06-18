using System.Collections;
using System.Collections.Generic;
using TMPro;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.AI;
using static UnityEditor.FilePathAttribute;

public class InsideSupermarket : MonoBehaviour
{
    // Contador de peatones
    private int pedestrianCount = 0;
    private TextMeshPro counterText;
    //public Transform inside;

    private void Awake()
    {
        counterText = GetComponentInChildren<TextMeshPro>();
        
    }
    private void Update()
    {
        counterText.text = "Shopping: " + pedestrianCount;
    }

    // Método llamado cuando otro collider entra en el trigger
    private void OnTriggerEnter(Collider other)
    {
        NavMeshAgent agent = other.GetComponent<NavMeshAgent>();
        if (agent != null)
        {
            PedestrianBehaviour pedestrian = agent.GetComponent<PedestrianBehaviour>();
            if (pedestrian != null)
            {
                agent.isStopped = true;
            }
        }
        pedestrianCount++;
        Debug.Log(pedestrianCount);
    }

    // Método llamado cuando otro collider sale del trigger
    private void OnTriggerExit(Collider other)
    {
        pedestrianCount--;
    }
   
 
}
