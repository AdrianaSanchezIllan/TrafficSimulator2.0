using UnityEngine;
using UnityEngine.AI;
using System.Collections;
using BehaviourAPI.Core;
using BehaviourAPI.StateMachines;
using BehaviourAPI.BehaviourTrees;
using System.Collections.Generic;
using static HoolBehaviour;
using static UnityEditor.ShaderData;
using System.Runtime.ConstrainedExecution;
using UnityEditor;
using Unity.VisualScripting;

public class HoolBehaviour : MonoBehaviour
{
    public NavMeshAgent agent;
    public float range; // Radio de deambulaci�n
    public Transform centrePoint; // Centro del �rea de deambulaci�n
    public Transform destinyPoint; // Punto destino: stadium
    private H_InterestLocation destLocation;
    public Vector3 destPos;

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
    private static readonly int isWarmingUp = Animator.StringToHash("isWarmingUp");
    private static readonly int isRunning = Animator.StringToHash("isRunning");
    private static readonly int isDancing = Animator.StringToHash("isDancing");

    private GameObject currentActionIndicator;
    public H_InterestZone currentZone;
    public string currentAction;
    private bool isActionCompleted = false;
    public H_InterestLocation currentTarget = null;
    H_InterestLocation runTo = null;

    private bool inTrafficLight = false;
    private bool isCrossingStreet = false;

    private bool checkReadyToGo = false;
    private bool goneToStadium = false;
    private bool enableMovement = false;
    public int timeToGo;
    
    private void Awake()
    {
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();
        destLocation = destinyPoint.GetComponent<H_InterestLocation>();
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
        if (destinyPoint == null)
        {
            destinyPoint = transform;
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
    

        

    
    IEnumerator WaitAndExecute(int t)
    {
        yield return new WaitForSeconds(t); // Esperar 5 segundos
        checkReadyToGo = true; // Ejecutar la acción
        Debug.Log("2-------> WAITANDEXECUTE");
    }
    #region Stadium
    public void StartDium()
    {
        Debug.Log("4-------> STARTDIUM");
        StartCoroutine(GoToStadium());
    }


    IEnumerator GoToStadium()
    {
        //Debug.Log("2------->Entra en gtstad");
    
        //MoveTo(destLocation);
        //A la fuerza
        //agent.isStopped = false;
        //agent.SetDestination(destLocation.transform.position);
        agent.isStopped = false;
        if (destPos != null) agent.SetDestination(destPos);
        else Debug.Log("5-------->No hay destino para este hooliigan");
        
            /*if (agent.remainingDistance <= agent.stoppingDistance)
            {
                Vector3 point;
                if (DestPoint(destinyPoint.position, 1.0f, out point))
                {
                    Debug.Log("5----->RandomPoint devuelve true");
                    Debug.DrawRay(point, Vector3.up, Color.blue, 1.0f);
                    agent.SetDestination(point);
                }
            }*/
        Debug.Log("5------->destino GOTOSTADIUM conseguido");

        bool isMoving = agent.velocity.magnitude > 0.01f;
        if (isStopped && !isCrossing)
        {
            agent.isStopped = true;
        }
        /*if(currentTarget != null && !isActionCompleted)
        {
            agent.SetDestination(currentTarget.transform.position);
        }*/
        animator.SetBool(IsWalking, isMoving);
        yield return null;
        
    }

    public bool CheckReadyToGo()
    {
        if (checkReadyToGo)
        {
            Debug.Log("3-------> CHECKREADYTOGO");
            checkReadyToGo = false;
            return true;
        }
        //Debug.Log("Esta en ZONA INTERES-----> Stadium");
        //currentZone = null;
        return false;
    }
    #endregion

    #region ROAMING
    // Estado de deambulaci�n
    public void StartRoaming()
    {
        Debug.Log("1----->Entra en start roaming");
        if (!goneToStadium){
            StartCoroutine(WaitAndExecute(timeToGo));
            goneToStadium = true;//Podría ir en otro lado
        }
        StartCoroutine(Roam());
    }

    private IEnumerator Roam()
    {
        Debug.Log("2------->Entra en Roam");
        while (true)
        {
            agent.isStopped = false;
            if (agent.remainingDistance <= agent.stoppingDistance)
            {
                Vector3 point;
                if (RandomPoint(centrePoint.position, range, out point))
                {
                    //Debug.Log("5----->RandomPoint devuelve true");
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
                if (enableMovement){
                    if (isPathComplete()){
                        currentZone.numFreePos--;
                        enableMovement = false;
                        Debug.Log("11,12--------->FUNCIONA?");
                    }

                }
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
            //Debug.Log("Esta en ZONA INTERES-----> Stadium");
            Debug.Log("7-------> ISINTERESTZONE");
            isActionCompleted = false;
            return true;
        }
        //Debug.Log("Esta en ZONA INTERES-----> Stadium");
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
        public Status IsInStadium()
    {
        if (currentAction == "Stadium")
        {
            Debug.Log("8-------> ISINSSTADIUM");
            return Status.Success;
        }
        Debug.Log("--->:(");
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
        if (currentZone.FreePosRemaining() <= 0) return Status.Success;
            agent.isStopped = true;
            if(  currentAction== "Stadium" || currentAction == "Coffee")
            {
                Debug.Log("9-------->SearchLocation");
                currentTarget = currentZone.GetFirstFreeLocation();
                enableMovement = true;
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
    private void SubstractPeoppleZone(){
        currentZone.numFreePos--;
    }
    public void MoveTo(H_InterestLocation newTarget)
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
        Debug.Log("11---->MOVETOLOCATION");
        MoveTo(currentTarget);
        
        if (HasReachedTarget())
        {
            this.currentZone.numFreePos = this.currentZone.numFreePos-1;
            Debug.Log("2------>Intetando restar posiciones");
            //H_InterestZone.numFreePos--;
            return Status.Success;
        }
        return Status.Failure;
    }

    public bool HasReachedTarget()
    {
        if (currentTarget != null)
        {
            Debug.Log("12-------> HASREACHEDTARGET");
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
            }
            if(currentAction == "Stadium")
            {
                Debug.Log("13-------> PERFORMACTIONBT");
                StartCoroutine(StadiumAction());
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

            yield return new WaitForSeconds(10.0f); // 10 segundos para tomar caf�
            
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
    IEnumerator StadiumAction()
    {
        if (!isActionCompleted)
        {
            if (currentZone.FreePosRemaining() <= 0){
                isActionCompleted = true;
                yield return null;
            } 
            else{
                while (agent.remainingDistance > agent.stoppingDistance)
                {
                    yield return null;
                }
                Debug.Log("14-------> STADIUMACTION");
                agent.isStopped = true;
                //Debug
                PerformAction(currentZone.interestAction);
                //Espera hasta que estén todos
                //while (currentZone.FreePosRemaining() < currentZone.locations.Count)
                //while (currentZone.FreePosRemaining() < currentZone.locations.Count)
                while (currentZone.FreePosRemaining() > 0)
                {
                    Debug.Log("ESPERANDO.... POSICIONES LIBRES " + currentZone.FreePosRemaining());
                    //Debug.Log("ESPERANDO.... POSICIONES OCUPADAS " + currentZone.locations.Count);
                    yield return new WaitForSeconds(1.0f); // Verifica cada 0.1 segundos
                }

                if (currentZone.actionIndicatorPrefab != null)
                {
                    if (currentActionIndicator == null)
                    {
                        currentActionIndicator = Instantiate(currentZone.actionIndicatorPrefab, transform);
                        currentActionIndicator.transform.localPosition = new Vector3(0, 3.0f, 0);
                        currentActionIndicator.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                    }
                    
                }
                animator.SetBool(isDancing, true);
                Debug.Log("Todos a bailarrrrrr");
                yield return new WaitForSeconds(10.0f); // 10 segundos para bailar
                animator.SetBool(isDancing, false);
                
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
        if (!isActionCompleted)
        {
            while (agent.remainingDistance > agent.stoppingDistance)
            {
                yield return null;
            }

            agent.isStopped = true;
            PerformAction(currentZone.interestAction);

            yield return new WaitForSeconds(10.0f); // Simular tiempo dentro de la tienda
            
            if (currentZone.actionIndicatorPrefab != null)
            {
                if (currentActionIndicator == null)
                {
                    currentActionIndicator = Instantiate(currentZone.actionIndicatorPrefab, transform);
                    currentActionIndicator.transform.localPosition = new Vector3(0, 2.0f, 0);
                    currentActionIndicator.transform.localScale = new Vector3(0.2f, 0.2f, 0.2f);
                }
            }

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
            case "Stadium":
                Debug.Log("15------->Messssssiiiii");
                break;
            default:
                Debug.LogWarning("Acci�n desconocida: " + action);
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
    private bool DestPoint(Vector3 center, float range, out Vector3 result)
    {
        Debug.Log("3------->Entra en DestPoint");
        Vector3 destPoint = center + Random.insideUnitSphere * range;
        NavMeshHit hit;
        if (NavMesh.SamplePosition(destPoint, out hit, 1.0f, NavMesh.AllAreas))
        {
            Debug.Log("4---------->Encuentra Posición");
            result = hit.position;
            return true;
        }

        result = Vector3.zero;
        return false;
    }
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
