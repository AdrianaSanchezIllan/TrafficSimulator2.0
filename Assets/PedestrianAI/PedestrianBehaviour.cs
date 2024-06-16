using UnityEngine;
using UnityEngine.AI;
using System.Collections;
using BehaviourAPI.Core;
using BehaviourAPI.StateMachines;
using BehaviourAPI.BehaviourTrees;
using System.Collections.Generic;
using static PedestrianBehaviour;
using static UnityEditor.ShaderData;
using System.Runtime.ConstrainedExecution;

public class PedestrianBehaviour : MonoBehaviour
{
    public NavMeshAgent agent;
    public float range; // Radio de deambulaci�n
    public Transform centrePoint; // Centro del �rea de deambulaci�n

    private List<Crosswalk> crosswalks = new List<Crosswalk>(); // Lista de pasos de cebra
    private Crosswalk nearbyCrosswalk; // Variable para almacenar el paso de cebra cercano
    private bool hasCrossed = false;
    private Vector3 originalDestination; // Para almacenar el destino original antes de cruzar
    private bool canCrossNow = false;
    private bool isStopped = false;
    private bool isInCrosswalk = false; // Flag para controlar la entrada y salida
                                        //public TrafficLights trafficLight; // Referencia al sem�foro
    private bool isCrossing = false;

    private Animator animator;
    private static readonly int IsWalking = Animator.StringToHash("isWalking");

    private GameObject currentActionIndicator;
    public InterestZone currentZone;
    public string currentAction;
    private bool isActionCompleted = false;
    public InterestLocation currentTarget = null;

    private bool inTrafficLight = false;
    private bool isCrossingStreet = false;
    
    private void Awake()
    {
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();

        if (animator == null)
        {
            Debug.LogError("Animator not found!");
        }

        if (agent == null)
        {
            agent = gameObject.AddComponent<NavMeshAgent>();
        }

        if (centrePoint == null)
        {
            centrePoint = transform;
        }

        // Buscar todos los objetos con la etiqueta "Crosswalk" y a�adir sus componentes a la lista
        GameObject[] crosswalkObjects = GameObject.FindGameObjectsWithTag("Crosswalk");
        foreach (var crosswalkObject in crosswalkObjects)
        {
            Crosswalk crosswalk = crosswalkObject.GetComponent<Crosswalk>();
            if (crosswalk != null)
            {
                crosswalks.Add(crosswalk);
            }
        }
    }
    #region ROAMING
    // Estado de deambulaci�n
    public void StartRoaming()
    {
        StartCoroutine(Roam());
    }

    private IEnumerator Roam()
    {
        while (true)
        {
            agent.isStopped = false;
            if (agent.remainingDistance <= agent.stoppingDistance)
            {
                Vector3 point;
                if (RandomPoint(centrePoint.position, range, out point))
                {
                    Debug.DrawRay(point, Vector3.up, Color.blue, 1.0f);
                    agent.SetDestination(point);
                }
            }

            bool isMoving = agent.velocity.magnitude > 0.1f;
            if (isStopped && !isCrossing)
            {
                agent.isStopped = true;
            }
            if(currentTarget != null && !isActionCompleted)
            {
                agent.SetDestination(currentTarget.transform.position);
            }
            animator.SetBool(IsWalking, isMoving);
            yield return null;

        }
    }
    #endregion

    #region CROSSWALK
    // Transicion para verificar si el peat�n en alg�n paso de cebra
    public bool InCrosswalkArea()
    {
        if(nearbyCrosswalk != null)
        {
            Debug.Log("En paso de cebra" + nearbyCrosswalk);
            return true;
        }
        nearbyCrosswalk = null;
        return false;
    }

    //guarda el paso de cebra en el q esta
    public void EnterCrosswalkArea(Crosswalk crosswalk)
    {
        if (!isInCrosswalk) // Verificar si ya est� en el paso de cebra
        {
            isInCrosswalk = true;
            nearbyCrosswalk = crosswalk;
            Debug.Log("Entered crosswalk area" + nearbyCrosswalk);
        }
    }

    public bool FinishCrossing()
    {
        if (!isInCrosswalk)
        {
            nearbyCrosswalk = null;
            return true;
        }
        return false;
    }

    public void ExitCrosswalkArea(Crosswalk crosswalk)
    {
        if (isInCrosswalk && crosswalk == nearbyCrosswalk)
        {
            isInCrosswalk = false;
            nearbyCrosswalk = null;
            Debug.Log("Exited crosswalk area");
        }
    }

    // M�todo para mover al peat�n al inicio del paso de cebra
    public void WaitUntilGreen()
    {
        Debug.Log("WAITINGGG" + nearbyCrosswalk);
        StartCoroutine(WaitAtCrosswalk());
        
    }


    // Estado para esperar en el paso de cebra
    IEnumerator WaitAtCrosswalk()
    {
        while (true)
        {
            isStopped = true; // Detener al agente
            isCrossing = false;
            //para comrpobar si nearbyCrosswalk es null
            if (nearbyCrosswalk == null)
            {
                Debug.LogError("nearbyCrosswalk is null");
                yield break;
            }
            //para comrpobar si trafficLight es null
            if (nearbyCrosswalk.trafficLight == null)
            {
                Debug.LogError("trafficLight is null");
                yield break;
            }

            if (nearbyCrosswalk.trafficLight.IsRedForCars())
            {
                isStopped = false; // Reanudar el agente
                canCrossNow = true;
                yield break;
            }

            yield return null; 
        }
    }

    public bool CanCrossNow()
    {
        isStopped = false;
        return canCrossNow;
    }
    // Estado para comenzar a cruzar la calle
    public void StartCrossing()
    {
        StartCoroutine(CrossStreet());
    }

    IEnumerator CrossStreet()
    {
        Debug.Log("CROSSINGGG" + nearbyCrosswalk);
        while (true)
        {
            isStopped = false;
            isCrossing = true;
            canCrossNow = true;
            if (FinishCrossing())
            {
                canCrossNow = false;
                isCrossing = false;
                yield break;
            }

            yield return null; 
        }
    }

    #endregion



    #region INTEREST ZONE
    // transicion para entrar en BT de acciones
    public bool IsInterestZone()
    {
        if (currentZone != null && !isCrossing)
        {
            Debug.Log("Esta en ZONA INTERES");
            return true;
        }
        currentZone = null;
        return false;
    }

    public Status SearchLocation()
    {
        Debug.Log("Buscar silla disponible");
        agent.isStopped = true;
        currentTarget = currentZone.GetFirstFreeLocation();

        if (currentTarget != null)
        {
            Debug.Log("Se asigna silla " + currentTarget);
            return Status.Success;
        }
        return Status.Failure;
    }

    public void MoveTo(InterestLocation newTarget)
    {
        if (newTarget != null)
        {
            currentTarget = newTarget;
            agent.SetDestination(currentTarget.transform.position);
        }
        else
        {
            Debug.LogError("New target is null.");
        }
    }

    public Status MoveToLocation()
    {
        Debug.Log("MOVING TO CHAIR");
        MoveTo(currentTarget);
        
        if (HasReachedTarget())
        {
            Debug.Log("REACHED THE CHAIR");
            return Status.Success;
        }
        return Status.Failure;
    }

    public bool HasReachedTarget()
    {
        if (currentTarget != null)
        {
            // Check if the agent has reached the target
            return !agent.pathPending && agent.remainingDistance <= agent.stoppingDistance;
        }
        return false;
    }

    public Status PerformActionBT()
    {
        if (!isActionCompleted)
        {
            StartCoroutine(PerformInterestAction());
            return Status.Running;
        }
        return Status.Success;
    }

    IEnumerator PerformInterestAction()
    {
        while (agent.remainingDistance > agent.stoppingDistance)
        {
            yield return null;
        }

        agent.isStopped = true;
        PerformAction(currentZone.interestAction);

        if (currentActionIndicator != null)
        {
            Destroy(currentActionIndicator);
        }

        if (currentZone.actionIndicatorPrefab != null)
        {
            currentActionIndicator = Instantiate(currentZone.actionIndicatorPrefab, transform);
            currentActionIndicator.transform.localPosition = new Vector3(0, 3.0f, 0);
            currentActionIndicator.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
            currentActionIndicator.AddComponent<RotateIndicator>();
        }

        // Simular duraci�n de la acci�n
        yield return new WaitForSeconds(10.0f); // Por ejemplo,10 segundos para tomar caf�

        if (currentActionIndicator != null)
        {
            Destroy(currentActionIndicator);
        }

        agent.isStopped = false;

        // Verificaci�n de currentZone antes de llamar a VacateLocation
        if (currentZone == null)
        {
            Debug.LogError("currentZone is null");
            yield break;
        }
        currentZone.VacateLocation(currentTarget);
        currentZone = null;
        currentTarget = null;
        isActionCompleted = true;
    }
    // Acciones en zonas de inter�s
    public void PerformAction(string action)
    {
        switch (action)
        {
            case "Coffee":
                Debug.Log("Tomando un caf�.");
                break;
            case "Park":
                Debug.Log("Dando una vuelta corriendo.");
                break;
            case "Supermarket":
                Debug.Log("Comprando v�veres.");
                break;
            default:
                Debug.LogWarning("Acci�n desconocida: " + action);
                break;
        }
    }

    public Status IsInCoffeeShop()
    {
        Debug.Log("compueba si es coffee shop");
        if (currentAction == "Coffee")
        {
            Debug.Log("Esta en una COFFEE SHOP");
            return Status.Success;
        }
        return Status.Failure;
    }

    public Status IsInPark()
    {
        if (currentAction == "Park")
        {
            Debug.Log("Esta en un PARK");
            return Status.Success;
        }
        return Status.Failure;
    }

    public Status IsInSupermarket()
    {
        if (currentAction == "Supermarket")
        {
            Debug.Log("Esta en un SUPERMARKET");
            return Status.Success;
        }
        return Status.Failure;
    }

    public bool HasCompletedAction()
    {
        return isActionCompleted;
    }

    public bool isPathComplete()
    {
        return (!agent.pathPending &&
                agent.remainingDistance <= agent.stoppingDistance &&
                (!agent.hasPath || agent.velocity.sqrMagnitude == 0f));
    }

    public Status PathStatus()
    {
        if (!agent.pathPending &&
            agent.remainingDistance <= agent.stoppingDistance &&
            (!agent.hasPath || agent.velocity.sqrMagnitude == 0f))
        {
            return Status.Success;
        }
        return Status.Running;
    }


    #endregion
    private bool RandomPoint(Vector3 center, float range, out Vector3 result)
    {
        Vector3 randomPoint = center + Random.insideUnitSphere * range;
        NavMeshHit hit;
        if (NavMesh.SamplePosition(randomPoint, out hit, 1.0f, NavMesh.AllAreas))
        {
            result = hit.position;
            return true;
        }

        result = Vector3.zero;
        return false;
    }
}
