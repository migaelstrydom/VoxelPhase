use super::*;

/// Every scenario holds to the invariants and does what it says it does.
#[test]
fn every_scenario_passes() {
    let mut failures = Vec::new();
    for scenario in catalogue() {
        let run = run(&scenario).expect("the scenario's level builds");
        let verdict = scenario.verdict(&run);
        if verdict.is_err() {
            failures.push(report(&run, &verdict));
        }
    }
    assert!(failures.is_empty(), "\n{}", failures.join("\n"));
}

#[test]
fn scenario_names_are_unique() {
    let names: Vec<&str> = catalogue().iter().map(|s| s.name).collect();
    for name in &names {
        assert_eq!(
            names.iter().filter(|n| *n == name).count(),
            1,
            "{name} twice"
        );
    }
}
