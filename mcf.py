# coding: utf-8

from flask import Flask, request
import json
import time

from ortools.linear_solver import pywraplp

# TODO make a copy of this template into a new file.
# TODO open Dockerfile and at the bottom  replace "template.py" with the name of your application's python file


class Worker(object):

    def __init__(self):
        # This method runs when a new worker is created.
        # You can create object variables here
        # TODO replace line below with your own code
        self._demo_variable = "Hello World"

    def run(self, received_json_as_dict):
        # This is the entry point for your specific code
        # TODO replace the rest of this method with your own code.
        # Return a single object.
        K = received_json_as_dict['K']
        nodes = received_json_as_dict['nodes']
        nodesP = received_json_as_dict['nodesP']
        nodesM = received_json_as_dict['nodesM']
        edgesS = received_json_as_dict['edgesS']
        edgesD = received_json_as_dict['edgesD']
        edgesW = received_json_as_dict['edgesW']
        edgesF = received_json_as_dict['edgesF']
        source = received_json_as_dict['source']
        destination = received_json_as_dict['destination']
        minP = received_json_as_dict['minP']
        maxP = received_json_as_dict['maxP']
        minF = received_json_as_dict['minF']
        maxF = received_json_as_dict['maxF']
        Excluded = received_json_as_dict['Excluded']
        ResultsCost = []
        ResultsRoutes = []
        SolutionErrorMessage = ''
        SolutionErrorCode = 0
        start = time.time()
        i = 1
        Results = []
        while i <= K:
            cost, R, D, errorCode, errorMessage = self.opt(nodes, nodesP, nodesM, edgesS, edgesD, edgesW, edgesF, source, destination, minP, maxP, minF, maxF, Excluded, start)
            if cost == -2:
                SolutionErrorMessage = errorMessage
                SolutionErrorCode = errorCode
                break
            if cost == -1:
                Excluded.append({'ExcludedItem': R})
            elif cost == 0:
                print("could find no more")
                break
            else:
                Excluded.append({'ExcludedItem': R})
                ResultsCost.append(cost)
                ResultsRoutes.append(R)
                Results.append({'nodes': D, 'cost': cost})
                i = i + 1

        return {"Results": Results,
                "Excluded": Excluded,
                "ErrorCode": SolutionErrorCode,
                "ErrorMessage": SolutionErrorMessage}

    def opt(self, nodes, nodesP, nodesM, edgesS, edgesD, edgesW, edgesF, source, destination, minP, maxP, minF, maxF, Excluded, start):

        # Input parameters

        # Solving Assignment Problem with MIP
        # Instantiate a mixed-integer solver.
        solver = pywraplp.Solver('SolveAssignmentProblemMIP', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
        remainungTime = round((30 - (time.time() - start)) * 1000)
        if remainungTime < 0:
            return -2, 0, 0, sol, 'out of time'

        solver.SetTimeLimit(remainungTime)
        num_edges = len(edgesS)
        x = {}
        for i in range(num_edges):
            x[i] = solver.BoolVar('x[%i]' % (i))

        # Objective
        solver.Minimize(solver.Sum([edgesW[i] * x[i] for i in range(num_edges)]))

        # Constraints
        for k in nodes:
            if k == source:
                solver.Add(solver.Sum([1 * x[i] for i in [i for i, val in enumerate(edgesS) if val == k]]) == 1)
            elif k == destination:
                solver.Add(solver.Sum([x[i] for i in [i for i, val in enumerate(edgesD) if val == k]]) == 1)
            else:
                solver.Add(solver.Sum([solver.Sum([x[i] for i in [i for i, val in enumerate(edgesD) if val == k]]),
                                       solver.Sum([-1 * x[i] for i in [i for i, val in enumerate(edgesS) if val == k]])]) == 0)

        if minP > -1:
            a = [i for i in [i for i, val in enumerate(nodesP) if val == 1]]
            b = [nodes[i] for i in a]
            solver.Add(solver.Sum([-1 * x[i] for i in [i for i, val in enumerate(edgesD) if val in b]]) <= -1 * minP)

        if maxP > -1:
            a = [i for i in [i for i, val in enumerate(nodesP) if val == 1]]
            b = [nodes[i] for i in a]
            solver.Add(solver.Sum([x[i] for i in [i for i, val in enumerate(edgesD) if val in b]]) <= maxP)

        if minF > -1:
            solver.Add(solver.Sum([-1 * x[i] for i in [i for i, val in enumerate(edgesF) if val == 1]]) <= -1 * minF)

        if maxF > -1:
            solver.Add(solver.Sum([x[i] for i in [i for i, val in enumerate(edgesF) if val == 1]]) <= maxF)

        if len(nodesM) > 0:
            for m in nodesM:
                solver.Add(solver.Sum([x[i] for i in [i for i, val in enumerate(edgesD) if val == m]]) == 1)

        for i in range(num_edges):
            if edgesS[i] < edgesD[i]:
                for j in range(num_edges):
                    if (edgesS[i] == edgesD[j]) & (edgesD[i] == edgesS[j]):
                        solver.Add(x[i] + x[j] <= 1)
                        break

        for i in range(len(Excluded)):
            solver.Add(solver.Sum([x[i] for i in Excluded[i]['ExcludedItem']]) <= len(Excluded[i]['ExcludedItem']) - 1)

        sol = solver.Solve()

        if sol == solver.OPTIMAL:
            z = 1
        elif sol == solver.FEASIBLE:
            return -2, 0, 0, sol, 'out of time'
        elif sol == solver.INFEASIBLE:
            return -2, 0, 0, sol, 'infeasible'
        elif sol == solver.ABNORMAL:
            return -2, 0, 0, sol, 'abnormal'
        elif sol == solver.NOT_SOLVED:
            return -2, 0, 0, sol, 'not solved'
        elif sol == solver.UNBOUNDED:
            return -2, sol, 0, sol, 'unbounded'

        R = []
        R2 = []
        S = []
        D = []
        D2 = [source]
        for i in range(len(edgesS)):
            if x[i].solution_value() == 1:
                R.append(i)
                R2.append(i)
                S.append(edgesS[i])
                D.append(edgesD[i])
                D2.append(edgesD[i])

        Results = [source]
        curr = source
        for i in range(len(D2) - 1):
            I = S.index(curr)
            curr = D[I]
            del S[I]
            del D[I]
            del R[I]
            if (curr == source or curr == destination) and i != len(R2) - 1:
                return -1, R, 0, sol, ''
            Results.append(curr)

        return solver.Objective().Value(), R2, Results, sol, ''


# This creates a the web server object. This uses Flask [1]
app = Flask(__name__)


def _create_http_response_ok(result):
    """
    Wraps the result parameter ready for returning to Flask
    :param result: something that can be converted to JSON
    :return: Just return this all to Flask: result_as_json_string, HTTP_OK = 200, Content-Type header
    """
    result_json = json.dumps(result, default=lambda obj: obj.__dict__ if '__dict__' in dir(obj) else "{}".format(obj))
    # Returning JSON-body, HTTP-response code, and the header telling the client it is JSON
    return result_json, 200, {'Content-Type': 'application/json'}


# This annotation defines an entry point at path / (root) for HTTP method POST. The Flask app will find this annotation
# and will forward matching requests to this function.
@app.route("/", methods=['POST'])
def root():
    """The client should send a JSON body and the header "Content-Type" with value "application/json".
    """
    # The Flask framework will parse the JSON string into a python dict and present it as request.json
    # The request is a thread local object not a parameter [2]
    if not request.json:
        return 'missing JSON body'
    received_json_as_dict = request.json

    # Creates a Worker object
    worker = Worker()

    # The worker should return an object that will be converted into a JSON string and send to the client
    result_dict = worker.run(received_json_as_dict)

    # Returning the body as a JSON string and the HTTP response code 200 meaning OK
    return _create_http_response_ok(result_dict)


# This annotation adds an entry point at path / for HTTP Get.
@app.route("/", methods=['GET'])
def health():
    """This function tells the web server is healthy."""
    return _create_http_response_ok({"message": "I am healthy"})


# This construct will only run if this script is started stand-alone (not when loaded as a module/AWS Lambda function)
if __name__ == "__main__":
    # This starts the web server for all client IP numbers, on port 80, allowing multiple threads [3], [4]
    app.run(host='0.0.0.0', port=80, threaded=True)

# References
# [1] Flask website http://flask.pocoo.org/
# [2] Flask Thread local http://flask.pocoo.org/docs/1.0/advanced_foreword/#thread-locals-in-flask
# [3] Flask run http://flask.pocoo.org/docs/1.0/api/#flask.Flask.run
# [4] Flask run options http://werkzeug.pocoo.org/docs/0.14/serving/#werkzeug.serving.run_simple

