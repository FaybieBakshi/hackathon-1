#!/usr/bin/env python3
"""
Generate detailed test reports and improvement suggestions for Spec 3.

This script runs comprehensive tests and analyzes the retrieval system's performance
to identify gaps in chunking/embedding quality and suggest improvements.
"""
import json
import os
from datetime import datetime
from typing import Dict, Any, List

from test_retrieval import run_comprehensive_tests, run_edge_case_tests
from benchmark import run_benchmark
from src.utils.validators import validate_performance_metrics


def generate_gaps_and_improvements_report() -> Dict[str, Any]:
    """
    Generate detailed test reports including gaps in chunking/embedding quality
    and create suggestions for improvements for Spec 3.
    """
    print("Generating comprehensive analysis report...")

    # Run comprehensive tests
    print("\n1. Running comprehensive accuracy tests...")
    accuracy_results = run_comprehensive_tests()

    # Run edge case tests
    print("\n2. Running edge case tests...")
    edge_case_results = run_edge_case_tests()

    # Run performance benchmarks
    print("\n3. Running performance benchmarks...")
    performance_results = run_benchmark(
        queries=accuracy_results["all_results"][:5],  # Use first 5 queries for performance test
        iterations=5
    )

    # Analyze gaps in chunking/embedding quality
    print("\n4. Analyzing gaps in chunking/embedding quality...")
    gaps_analysis = analyze_chunking_embedding_gaps(accuracy_results)

    # Generate improvement suggestions for Spec 3
    print("\n5. Generating improvement suggestions...")
    improvement_suggestions = generate_improvement_suggestions(
        accuracy_results,
        performance_results,
        gaps_analysis
    )

    # Compile full report
    report = {
        "timestamp": datetime.now().isoformat(),
        "accuracy_results": accuracy_results,
        "edge_case_results": edge_case_results,
        "performance_results": performance_results,
        "gaps_analysis": gaps_analysis,
        "improvement_suggestions": improvement_suggestions,
        "summary": {
            "overall_accuracy": accuracy_results["accuracy_rate"],
            "accuracy_target_met": accuracy_results["summary"]["target_met"],
            "edge_cases_passed": edge_case_results["all_passed"],
            "performance_targets_met": performance_results["summary"]["targets_met"]
        }
    }

    return report


def analyze_chunking_embedding_gaps(accuracy_results: Dict[str, Any]) -> Dict[str, Any]:
    """
    Analyze potential gaps in chunking/embedding quality based on test results.

    Args:
        accuracy_results: Results from comprehensive tests

    Returns:
        Dictionary with analysis of potential gaps
    """
    gaps = {
        "low_accuracy_categories": [],
        "common_failure_patterns": [],
        "potential_chunking_issues": [],
        "embedding_quality_concerns": []
    }

    # Identify categories with lower accuracy
    for category, stats in accuracy_results["category_breakdown"].items():
        if stats["accuracy_rate"] < 0.90:  # Lower threshold for identifying issues
            gaps["low_accuracy_categories"].append({
                "category": category,
                "accuracy_rate": stats["accuracy_rate"],
                "total_queries": stats["total"],
                "accurate_queries": stats["accurate"]
            })

    # Analyze common failure patterns
    failed_queries = []
    for result in accuracy_results["all_results"]:
        if not result["is_accurate"]:
            failed_queries.append({
                "query": result["query"],
                "status": result["result"].status,
                "num_chunks": len(result["result"].chunks)
            })

    # If we have failed queries, analyze patterns
    if failed_queries:
        # Look for patterns in failed queries
        empty_results = [q for q in failed_queries if q["num_chunks"] == 0]
        low_confidence = [q for q in failed_queries if q["status"] == "low_confidence"]

        if empty_results:
            gaps["common_failure_patterns"].append({
                "pattern": "no_results_returned",
                "count": len(empty_results),
                "examples": empty_results[:3]  # Show first 3 examples
            })

        if low_confidence:
            gaps["common_failure_patterns"].append({
                "pattern": "low_confidence_matches",
                "count": len(low_confidence),
                "examples": low_confidence[:3]
            })

    # Potential chunking issues
    gaps["potential_chunking_issues"] = [
        "Chunks might be too small, missing context",
        "Chunks might be too large, diluting relevance",
        "Chunk boundaries might break semantic coherence",
        "Overlap between chunks might be insufficient"
    ]

    # Potential embedding quality concerns
    gaps["embedding_quality_concerns"] = [
        "Embeddings might not capture semantic meaning effectively",
        "Query-document alignment might be suboptimal",
        "Domain-specific terminology might not be well represented",
        "Long-tail concepts might not be well embedded"
    ]

    return gaps


def generate_improvement_suggestions(
    accuracy_results: Dict[str, Any],
    performance_results: Dict[str, Any],
    gaps_analysis: Dict[str, Any]
) -> Dict[str, Any]:
    """
    Generate improvement suggestions for Spec 3 based on analysis.

    Args:
        accuracy_results: Results from accuracy tests
        performance_results: Results from performance benchmarks
        gaps_analysis: Analysis of gaps in current system

    Returns:
        Dictionary with improvement suggestions
    """
    suggestions = {
        "chunking_improvements": [],
        "embedding_improvements": [],
        "retrieval_improvements": [],
        "performance_improvements": [],
        "validation_improvements": []
    }

    # Chunking improvements
    suggestions["chunking_improvements"] = [
        "Implement semantic-aware chunking to preserve context",
        "Experiment with different chunk sizes (200, 400, 800 tokens)",
        "Add overlap between chunks to maintain continuity",
        "Use sentence boundaries to avoid breaking semantic units",
        "Implement content-aware chunking based on document structure"
    ]

    # Embedding improvements
    suggestions["embedding_improvements"] = [
        "Try different embedding models (e.g., multilingual models)",
        "Implement query-aware embedding generation",
        "Use fine-tuned embeddings for domain-specific content",
        "Experiment with different input types (document vs query optimization)",
        "Add post-processing to embeddings to improve semantic alignment"
    ]

    # Retrieval improvements
    suggestions["retrieval_improvements"] = [
        "Implement hybrid search (keyword + semantic)",
        "Add reranking after initial retrieval",
        "Use multi-vector retrieval for better matching",
        "Implement query expansion techniques",
        "Add negative mining to improve relevance"
    ]

    # Performance improvements
    suggestions["performance_improvements"] = [
        "Implement caching for frequent queries",
        "Add query result pre-computation for common queries",
        "Optimize vector database indexing",
        "Implement approximate nearest neighbor search",
        "Add query parallelization for batch processing"
    ]

    # Validation improvements
    suggestions["validation_improvements"] = [
        "Add more diverse test queries covering edge cases",
        "Implement human evaluation for relevance assessment",
        "Create domain-specific test sets",
        "Add adversarial test queries",
        "Implement continuous monitoring of retrieval quality"
    ]

    return suggestions


def save_report(report: Dict[str, Any], filename: str = None):
    """
    Save the analysis report to a file.

    Args:
        report: The report dictionary to save
        filename: Optional filename (defaults to timestamped name)
    """
    if filename is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"retrieval_analysis_report_{timestamp}.json"

    with open(filename, 'w', encoding='utf-8') as f:
        json.dump(report, f, indent=2, ensure_ascii=False, default=str)

    print(f"\nReport saved to: {filename}")


def print_summary_report(report: Dict[str, Any]):
    """
    Print a summary of the analysis report.

    Args:
        report: The report dictionary to summarize
    """
    summary = report["summary"]

    print("\n" + "="*70)
    print("RAG RETRIEVAL SYSTEM ANALYSIS REPORT SUMMARY")
    print("="*70)

    print(f"\nAccuracy Metrics:")
    print(f"  Overall Accuracy: {summary['overall_accuracy']:.2%}")
    print(f"  Target Met: {summary['accuracy_target_met']}")

    print(f"\nEdge Case Handling:")
    print(f"  All Tests Passed: {summary['edge_cases_passed']}")

    print(f"\nPerformance:")
    print(f"  Targets Met: {summary['performance_targets_met']}")
    print(f"  Cached P95 Latency: {report['performance_results']['cached_metrics']['p95_latency']:.2f}ms")
    print(f"  Cold Start P95 Latency: {report['performance_results']['cold_start_metrics']['p95_latency']:.2f}ms")

    print(f"\nGaps Identified:")
    print(f"  Low accuracy categories: {len(report['gaps_analysis']['low_accuracy_categories'])}")
    print(f"  Common failure patterns: {len(report['gaps_analysis']['common_failure_patterns'])}")

    print(f"\nImprovement Suggestions:")
    print(f"  Chunking: {len(report['improvement_suggestions']['chunking_improvements'])} suggestions")
    print(f"  Embedding: {len(report['improvement_suggestions']['embedding_improvements'])} suggestions")
    print(f"  Retrieval: {len(report['improvement_suggestions']['retrieval_improvements'])} suggestions")
    print(f"  Performance: {len(report['improvement_suggestions']['performance_improvements'])} suggestions")
    print(f"  Validation: {len(report['improvement_suggestions']['validation_improvements'])} suggestions")

    print("="*70)


if __name__ == "__main__":
    # Generate comprehensive analysis report
    report = generate_gaps_and_improvements_report()

    # Print summary
    print_summary_report(report)

    # Save full report
    save_report(report)

    print(f"\nAnalysis complete! Full report saved with detailed findings.")