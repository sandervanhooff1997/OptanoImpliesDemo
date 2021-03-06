﻿using OPTANO.Modeling.Optimization;
using OPTANO.Modeling.Optimization.Configuration;
using OPTANO.Modeling.Optimization.Enums;
using OPTANO.Modeling.Optimization.Solver.GLPK;
using System;
using System.Collections.Generic;
using System.Linq;

namespace OptanoImplies
{
    class Program
    {
        static void Main(string[] args)
        {
            var config = new Configuration();
            config.NameHandling = NameHandlingStyle.UniqueLongNames;
            config.ComputeRemovedVariables = true;

            using (var scope = new ModelScope(config))
            {
                var model = new Model();

                // days in crop planning
                var horizon = Enumerable.Range(1, 9).ToList();

                // only 1 crop for demonstration purposes
                var crops = new List<string>();
                crops.Add("Crop A");
                crops.Add("Crop B");

                // the assignment of a crop planted on a day in the horizon
                var CropAssignment = new VariableCollection<int, string>(
                        model,
                        horizon,
                        crops,
                        "CropAssignment",
                        (d, c) => $"CropAssignment_{d}_{c}",
                        (d, c) => 0,
                        (d, c) => 1,
                        (d, c) => VariableType.Binary
                    );

                foreach (var day in horizon)
                {
                    // max 1 crop per day
                    model.AddConstraint(Expression.Sum(crops.Select(crop => CropAssignment[day, crop])) <= 1);

                    foreach (var crop in crops)
                    {
                        // let's say a crop grows for 2 days after the day it was planted
                        var cropGrowing = horizon.Where(d => d > day && d <= day + 2);

                        if (cropGrowing.Count() == 2)
                        {
                            // set otherDay to zero, if the crop is planted on day.
                            foreach (var otherDay in cropGrowing)
                            {
                                model.AddConstraint(-CropAssignment[day, crop] + 1 >= Expression.Sum(crops.Select(c => CropAssignment[otherDay, c])));
                            }
                        }
                        else
                        {
                            // the crop can't finish before end of horizon
                            var cropCantFinishGrowingConstraint = CropAssignment[day, crop] == 0;
                            model.AddConstraint(cropCantFinishGrowingConstraint);
                        }
                    }
                }

                var total = Expression.Sum(horizon.SelectMany(day => crops.Select(crop => CropAssignment[day, crop])));
                model.AddObjective(new Objective(total, "total", ObjectiveSense.Maximize));

                using (var solver = new GLPKSolver())   
                {
                    var solution = solver.Solve(model);

                    // import values back into the model
                    model.VariableCollections.ToList().ForEach(vc => vc.SetVariableValues(solution.VariableValues));

                    // print solution 
                    foreach (var day in horizon)
                        foreach (var crop in crops)
                        {
                            if (CropAssignment[day, crop].Value == 1)
                                Console.WriteLine($"Day {day} {crop} planted");
                        }
                }
            }
        }
    }
}
