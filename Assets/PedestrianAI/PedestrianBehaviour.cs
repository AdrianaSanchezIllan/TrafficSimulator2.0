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
using UnityEditor;
using Unity.VisualScripting;
using System.ComponentModel;

public class PedestrianBehaviour : MonoBehaviour
{
    public NavMeshAgent agent;
    public float range; // Radio de deambulación
    public Transform centrePoint; // Centro del área de deambulación

    private List<Crosswalk> crosswalks = new List<Crosswalk>(); // Lista de pasos de cebra
    private Crosswalk nearbyCrosswalk; // Variable para almacenar el paso de cebra cercano
    private bool hasCrossed = false;
    private Vector3 originalDestination; // Para almacenar el destino original antes de cruzar
    private bool canCrossNow = false;
    private bool isStopped = false;
    private bool isInCrosswalk = false; // Flag para controlar la entrada y salida
                                        //public TrafficLights trafficLight; // Referencia al semáforo
    private bool isCrossing = false;

    private Animator animator;
    private static readonly int IsWalking = Animator.StringToHash("isWalking");
    private static readonly int isWarmingUp = Animator.StringToHash("isWarmingUp");
    private static readonly int isRunning = Animator.StringToHash("isRunning");

    private GameObject currentActionIndicator;
    public InterestZone currentZone;
    public string currentAction;
    private bool isActionCompleted = false;
    public InterestLocation currentTarget = null;
    public InterestLocation runTo = null;
    public Transform insideSupermarket;
    public Transform outsideSupermarket;
    public bool hasFinishedShopping = false;
    private Vector3 randomAux;
    private bool canSpawnOutside = false;


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
        randomAux = new Vector3(Random.Range(0.5f, 3), 0, Random.Range(0.5f, 1));
        
        // Buscar todos los objetos con la etiqueta "Crosswalk" y añadir sus componentes a la lista
        GameObject[] crosswalkObjects = GameObject.FindGameObjectsWithTag("Crosswalk");
        foreach (var crosswalkObject in crosswalkObjects)
        {
            Crosswalk crosswalk = crosswalkObject.GetComponent<Crosswalk>();
            if (crosswalk != null)
            {
                crosswalks.Add(crosswalk);
            }
        }

        GameObject supermarketObject = GameObject.FindWithTag("Supermarket");
        if (supermarketObject != null)
        {
            insideSupermarket = supermarketObject.transform;
        }
        else
        {
            Debug.LogError("No se encontró el objeto del supermercado con la etiqueta 'Supermarket'.");
        }

        GameObject supermarketExit = GameObject.FindWithTag("Salida");
        if (supermarketExit != null)
        {
            outsideSupermarket = supermarketExit.transform;
        }
        else
        {
            Debug.LogError("No se encontró el objeto del supermercado con la etiqueta 'Salida'.");
        }

    }
    #region ROAMING
    // Estado de deambulación
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

            bool isMoving = agent.velocity.magnitude > 0.01f;
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
    // Transicion para verificar si el peatón en algún paso de cebra
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
        if (!isInCrosswalk) // Verificar si ya está en el paso de cebra
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

    // Método para mover al peatón al inicio del paso de cebra
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
        //Debug.Log("CROSSINGGG" + nearbyCrosswalk);
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
            //Debug.Log("Esta en ZONA INTERES");
            isActionCompleted = false;
            return true;
        }
        currentZone = null;
        return false;
    }
    //sitios donde puedes estar
    public Status IsInCoffeeShop()
    {
        //Debug.Log("compueba si es coffee shop");
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

    public Status SearchLocation()
    {
        agent.isStopped = true;
        if(currentAction == "Coffee")
        {
            currentTarget = currentZone.GetFirstFreeLocation();
        }
        if (currentAction == "Park" || currentAction == "Supermarket")
        {
            currentTarget = currentZone.GetClosestLocation(agent.transform);
        }

        if (currentTarget != null)
        {
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
        MoveTo(currentTarget);
        
        if (HasReachedTarget())
        {
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
            if(currentAction == "Coffee")
            {
                StartCoroutine(CoffeeAction());
            }
            if(currentAction == "Park")
            {
                StartCoroutine(ParkAction());
            }
            if(currentAction == "Supermarket")
            {
                StartCoroutine(SupermarketAction());
                if(hasFinishedShopping)
                {
                    return Status.Success;
                }
            }
            
            return Status.Running;
        }
        return Status.Success;
    }

    IEnumerator CoffeeAction()
    {
        if (!isActionCompleted)
        {
            while (agent.remainingDistance > agent.stoppingDistance)
            {
                yield return null;
            }

            agent.isStopped = true;
            PerformAction(currentZone.interestAction);

            if (currentZone.actionIndicatorPrefab != null)
            {
                if (currentActionIndicator == null)
                {
                    currentActionIndicator = Instantiate(currentZone.actionIndicatorPrefab, transform);
                    currentActionIndicator.transform.localPosition = new Vector3(0, 3.0f, 0);
                    currentActionIndicator.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                }
                
            }

            yield return new WaitForSeconds(10.0f); // 10 segundos para tomar café
            
            if (currentActionIndicator != null)
            {
                Destroy(currentActionIndicator);
            }

            agent.isStopped = false;
            currentZone.VacateLocation(currentTarget);
            currentZone = null;
            currentTarget = null;
            isActionCompleted = true;
        }
        
    }

    IEnumerator ParkAction()
    {
        if (!isActionCompleted)
        {
            while (agent.remainingDistance > agent.stoppingDistance)
            {
                yield return null;
            }

            agent.isStopped = true;
            PerformAction(currentZone.interestAction);

            if (currentZone.actionIndicatorPrefab != null)
            {
                if (currentActionIndicator == null)
                {
                    currentActionIndicator = Instantiate(currentZone.actionIndicatorPrefab, transform);
                    currentActionIndicator.transform.localPosition = new Vector3(0, 2.0f, 0);
                    currentActionIndicator.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                }
            }

            animator.SetBool(isWarmingUp, true);
            yield return new WaitForSeconds(5.0f);
            animator.SetBool(isWarmingUp, false);

            if (runTo == null)
            {
                runTo = currentZone.GetFurthestLocation(agent.transform);
                MoveTo(runTo);
                Debug.Log("Corre hacia el otro extremo: " + runTo.name);
                agent.speed = 3;
                animator.SetBool(isRunning, true);
                Debug.Log("CORRIENDO");
                // Esperar hasta que se complete el camino
                while (!isPathComplete())
                {
                    yield return null;
                }

                //yield return new WaitForSeconds(0.5f);

                if (currentActionIndicator != null)
                {
                    Destroy(currentActionIndicator);
                }
                animator.SetBool(isRunning, false);
                animator.SetBool(isWarmingUp, false);
                animator.SetBool(IsWalking, false);
                //currentZone.VacateLocation(runTo);
                agent.speed = 2;
                isActionCompleted = true;
                agent.isStopped = false;
                currentZone.VacateLocation(currentTarget);
                currentZone = null;
                //currentTarget = null;
            }
            
        }

    }

    IEnumerator SupermarketAction()
    {
        if (!hasFinishedShopping)
        {
            /*while (agent.remainingDistance > agent.stoppingDistance)
            {
                yield return null;
            }*/

            agent.isStopped = true;
            PerformAction(currentZone.interestAction);
            if (insideSupermarket != null)
            {
                agent.Warp(insideSupermarket.position);
                agent.isStopped = true;
                
                yield return new WaitForSeconds(10.0f); // Simular tiempo dentro de la tienda

                //agent.isStopped = false;    
            }
            else
            {
                Debug.LogError("InsideShop is null.");
            }
         
            //currentTarget = null;
            hasFinishedShopping = true;
        }

    }
    public Status FinishShoppingBT()
    {
        agent.isStopped = true;
        if (!isActionCompleted)
        {
            StartCoroutine(FinishShopping());
            return Status.Running;
        }
        
        return Status.Success;

    }
    IEnumerator FinishShopping()
    {
        if (!isActionCompleted)
        {
            agent.isStopped = true;

            if (!canSpawnOutside)
            {
                Vector3 outsideSpawn = outsideSupermarket.position + randomAux;
                agent.Warp(outsideSpawn); // spawnea fuera del super
                canSpawnOutside = true;
                currentTarget = null;
            }
            else
            {
                agent.isStopped = true;
                Debug.Log("SALE CON LA COMPRA");
                if (currentZone.actionIndicatorPrefab != null)
                {
                    if (currentActionIndicator == null)
                    {
                        currentActionIndicator = Instantiate(currentZone.actionIndicatorPrefab, transform);
                        currentActionIndicator.transform.localPosition = new Vector3(0, 2.0f, 0);
                        currentActionIndicator.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                    }
                }
                yield return new WaitForSeconds(5.0f);
                Debug.Log("Me voy");
                if (currentActionIndicator != null)
                {
                    Destroy(currentActionIndicator);
                }
                currentZone = null;
                
                canSpawnOutside = false;
                isActionCompleted = true;
            }
            
        }
    }

    // Acciones en zonas de interés
    public void PerformAction(string action)
    {
        switch (action)
        {
            case "Coffee":
                Debug.Log("Tomando un café.");
                break;
            case "Park":
                Debug.Log("Dando una vuelta corriendo.");
                break;
            case "Supermarket":
                Debug.Log("Comprando víveres.");
                break;
            default:
                Debug.LogWarning("Acción desconocida: " + action);
                break;
        }
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
