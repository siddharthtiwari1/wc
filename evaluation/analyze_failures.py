#!/usr/bin/env python3
"""
Failure Mode Analysis for RAN System

Analyzes failed navigation attempts to identify common failure patterns:
- Incorrect object selection
- Verification failures
- Navigation timeout
- Safety stops

Author: Siddharth Tiwari
"""

import argparse
import json
import pandas as pd
from pathlib import Path
from collections import Counter


class FailureAnalyzer:
    """Analyze navigation failures."""

    def __init__(self, results_dir: str):
        self.results_dir = Path(results_dir)
        self.results_csv = self.results_dir / 'results.csv'
        self.df = pd.read_csv(self.results_csv)

        print(f'Loaded {len(self.df)} experiments from {results_dir}')

    def analyze_subgoal_failures(self):
        """Identify which types of instructions fail most often."""
        print("\n" + "="*70)
        print("SUBGOAL FAILURE ANALYSIS")
        print("="*70)

        # Identify failed subgoals
        self.df['failed_subgoals'] = self.df['subgoals_attempted'] - self.df['subgoals_succeeded']

        # Group by instruction
        instruction_failures = self.df.groupby('instruction').agg({
            'failed_subgoals': 'sum',
            'subgoals_attempted': 'sum'
        })

        instruction_failures['failure_rate'] = (
            instruction_failures['failed_subgoals'] / instruction_failures['subgoals_attempted']
        )

        instruction_failures = instruction_failures.sort_values('failure_rate', ascending=False)

        print("\nTop 10 most challenging instructions:")
        print("-" * 70)
        for idx, (instruction, row) in enumerate(instruction_failures.head(10).iterrows(), 1):
            print(f"{idx}. {instruction[:60]}...")
            print(f"   Failure Rate: {row['failure_rate']:.1%} ({int(row['failed_subgoals'])}/{int(row['subgoals_attempted'])} subgoals)")

    def analyze_full_chain_failures(self):
        """Analyze multi-step instruction failures."""
        print("\n" + "="*70)
        print("FULL-CHAIN FAILURE ANALYSIS")
        print("="*70)

        # Filter multi-step instructions
        multi_step = self.df[self.df['subgoals_attempted'] > 1]

        if len(multi_step) == 0:
            print("No multi-step instructions found.")
            return

        # Group by complexity
        complexity_analysis = multi_step.groupby('subgoals_attempted').agg({
            'full_chain': lambda x: (x == True).sum(),
            'instruction': 'count'
        })

        complexity_analysis.columns = ['full_chains_completed', 'total_attempts']
        complexity_analysis['fcc'] = (
            complexity_analysis['full_chains_completed'] / complexity_analysis['total_attempts']
        )

        print("\nFull-Chain Completion by Complexity:")
        print("-" * 70)
        for n_subgoals, row in complexity_analysis.iterrows():
            print(f"{int(n_subgoals)}-step instructions: "
                  f"{row['fcc']:.1%} "
                  f"({int(row['full_chains_completed'])}/{int(row['total_attempts'])})")

    def analyze_method_comparison(self):
        """Compare failure modes across methods."""
        print("\n" + "="*70)
        print("METHOD COMPARISON")
        print("="*70)

        methods = self.df['method'].unique()

        print("\nFailure Statistics by Method:")
        print("-" * 70)

        for method in methods:
            method_df = self.df[self.df['method'] == method]

            total_subgoals = method_df['subgoals_attempted'].sum()
            failed_subgoals = total_subgoals - method_df['subgoals_succeeded'].sum()
            failure_rate = failed_subgoals / total_subgoals if total_subgoals > 0 else 0

            safety_violations = method_df['safety_violations'].sum()

            full_chain_attempts = len(method_df[method_df['subgoals_attempted'] > 1])
            full_chain_success = (method_df['full_chain'] == True).sum()

            print(f"\n{method.upper()}:")
            print(f"  Subgoal Failures: {failed_subgoals}/{total_subgoals} ({failure_rate:.1%})")
            print(f"  Full-Chain Failures: {full_chain_attempts - full_chain_success}/{full_chain_attempts}")
            print(f"  Safety Violations: {safety_violations}")

    def identify_attribute_confusion(self):
        """Identify if certain attributes cause more failures."""
        print("\n" + "="*70)
        print("ATTRIBUTE-SPECIFIC FAILURES")
        print("="*70)

        # Parse instructions for attributes
        color_keywords = ['red', 'blue', 'green', 'yellow', 'white', 'black', 'brown']
        shape_keywords = ['round', 'square', 'rectangular', 'curved']
        material_keywords = ['wooden', 'metal', 'plastic', 'glass', 'ceramic']

        attribute_failures = {
            'color': {},
            'shape': {},
            'material': {}
        }

        for _, row in self.df.iterrows():
            instruction = row['instruction'].lower()
            failed = row['subgoals_succeeded'] < row['subgoals_attempted']

            # Check for colors
            for color in color_keywords:
                if color in instruction:
                    if color not in attribute_failures['color']:
                        attribute_failures['color'][color] = {'total': 0, 'failed': 0}
                    attribute_failures['color'][color]['total'] += 1
                    if failed:
                        attribute_failures['color'][color]['failed'] += 1

            # Check for shapes
            for shape in shape_keywords:
                if shape in instruction:
                    if shape not in attribute_failures['shape']:
                        attribute_failures['shape'][shape] = {'total': 0, 'failed': 0}
                    attribute_failures['shape'][shape]['total'] += 1
                    if failed:
                        attribute_failures['shape'][shape]['failed'] += 1

            # Check for materials
            for material in material_keywords:
                if material in instruction:
                    if material not in attribute_failures['material']:
                        attribute_failures['material'][material] = {'total': 0, 'failed': 0}
                    attribute_failures['material'][material]['total'] += 1
                    if failed:
                        attribute_failures['material'][material]['failed'] += 1

        # Print results
        for attr_type, attrs in attribute_failures.items():
            if attrs:
                print(f"\n{attr_type.capitalize()} Attributes:")
                print("-" * 40)
                for attr, stats in sorted(attrs.items(), key=lambda x: x[1]['failed']/x[1]['total'] if x[1]['total'] > 0 else 0, reverse=True):
                    if stats['total'] > 0:
                        failure_rate = stats['failed'] / stats['total']
                        print(f"  {attr}: {failure_rate:.1%} ({stats['failed']}/{stats['total']} failed)")

    def generate_failure_report(self):
        """Generate comprehensive failure analysis report."""
        output_file = self.results_dir / 'failure_analysis.txt'

        with open(output_file, 'w') as f:
            # Redirect print to file
            import sys
            old_stdout = sys.stdout
            sys.stdout = f

            print("RAN SYSTEM FAILURE ANALYSIS REPORT")
            print("=" * 70)
            print(f"Results Directory: {self.results_dir}")
            print(f"Total Experiments: {len(self.df)}")
            print(f"Date: {pd.Timestamp.now().strftime('%Y-%m-%d %H:%M:%S')}")
            print()

            self.analyze_subgoal_failures()
            self.analyze_full_chain_failures()
            self.analyze_method_comparison()
            self.identify_attribute_confusion()

            # Recommendations
            print("\n" + "="*70)
            print("RECOMMENDATIONS")
            print("="*70)

            # Compute overall failure rate
            total_subgoals = self.df['subgoals_attempted'].sum()
            failed_subgoals = total_subgoals - self.df['subgoals_succeeded'].sum()
            overall_failure_rate = failed_subgoals / total_subgoals if total_subgoals > 0 else 0

            print(f"\nOverall Failure Rate: {overall_failure_rate:.1%}")

            if overall_failure_rate > 0.3:  # >30% failure
                print("\nHigh failure rate detected. Recommendations:")
                print("1. Check perception quality (blur, lighting)")
                print("2. Verify attribute extraction accuracy")
                print("3. Lower verification thresholds (level_2_threshold, level_3_threshold)")
                print("4. Increase training data for difficult attributes")
            elif overall_failure_rate > 0.15:  # 15-30%
                print("\nModerate failure rate. Recommendations:")
                print("1. Review failed instructions for common patterns")
                print("2. Fine-tune verification cascade thresholds")
                print("3. Improve multi-view fusion for ambiguous cases")
            else:
                print("\nLow failure rate - system performing well!")
                print("Continue monitoring edge cases and challenging scenarios.")

            sys.stdout = old_stdout

        print(f"\n✓ Failure analysis saved to: {output_file}")

    def run_analysis(self):
        """Run complete failure analysis."""
        self.analyze_subgoal_failures()
        self.analyze_full_chain_failures()
        self.analyze_method_comparison()
        self.identify_attribute_confusion()
        self.generate_failure_report()


def main():
    parser = argparse.ArgumentParser(description='Analyze RAN navigation failures')
    parser.add_argument('results_dir', type=str,
                       help='Path to results directory (contains results.csv)')

    args = parser.parse_args()

    analyzer = FailureAnalyzer(args.results_dir)
    analyzer.run_analysis()


if __name__ == '__main__':
    main()
