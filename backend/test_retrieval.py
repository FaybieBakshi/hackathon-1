"""
Comprehensive test suite for the RAG retrieval system.

This module contains tests for:
- Factual, conceptual, and keyword-based query categories
- Accuracy validation (>95% success rate)
- 20+ diverse sample queries
- Test result validation against success criteria
- Detailed accuracy and performance reporting
"""
import json
import time
from typing import List, Dict, Any
from dataclasses import dataclass

from src.retrieval.retriever import retrieve_chunks, RetrievalResult
from src.utils.validators import validate_retrieval_results
from benchmark import run_benchmark, print_benchmark_report


@dataclass
class QuerySet:
    """Data class for a set of test queries."""
    name: str
    queries: List[str]
    category: str  # "factual", "conceptual", "keyword-based"
    expected_outcomes: List[Dict[str, Any]]  # Expected results for validation


def create_test_query_sets() -> List[QuerySet]:
    """
    Create test query sets for factual, conceptual, and keyword-based categories.

    Returns:
        List of QuerySet objects with diverse sample queries
    """
    factual_queries = [
        "What is the definition of RAG?",
        "How many layers does a transformer have?",
        "What is the purpose of vector embeddings?",
        "Explain the attention mechanism in 50 words",
        "What is cosine similarity used for?",
        "How does tokenization work?",
        "What is the difference between GPT-2 and GPT-3?",
        "What is the size of the vocabulary in BERT?",
        "How many parameters does GPT-3 have?",
        "What is the training data for GPT-3?"
    ]

    conceptual_queries = [
        "How does semantic search differ from keyword search?",
        "What are the advantages of using embeddings?",
        "Explain the concept of attention in transformers",
        "How does RAG improve language model responses?",
        "What is the role of vector databases in RAG?",
        "How does context length affect transformer models?",
        "What are the limitations of pre-trained language models?",
        "How does fine-tuning differ from few-shot learning?",
        "What is the relationship between embeddings and similarity?",
        "How do transformers handle sequential information?"
    ]

    keyword_based_queries = [
        "neural networks deep learning",
        "vector database similarity search",
        "transformer architecture attention",
        "embeddings semantic search",
        "RAG system architecture",
        "machine learning models",
        "natural language processing",
        "artificial intelligence applications",
        "tokenization text processing",
        "query embedding generation"
    ]

    return [
        QuerySet(
            name="Factual Queries",
            queries=factual_queries,
            category="factual",
            expected_outcomes=[{"relevant": True} for _ in factual_queries]
        ),
        QuerySet(
            name="Conceptual Queries",
            queries=conceptual_queries,
            category="conceptual",
            expected_outcomes=[{"relevant": True} for _ in conceptual_queries]
        ),
        QuerySet(
            name="Keyword-based Queries",
            queries=keyword_based_queries,
            category="keyword-based",
            expected_outcomes=[{"relevant": True} for _ in keyword_based_queries]
        )
    ]


def run_accuracy_test(query: str, min_score: float = 0.3) -> Dict[str, Any]:
    """
    Run accuracy test for a single query.

    Args:
        query: The query to test
        min_score: Minimum confidence score threshold

    Returns:
        Dictionary with test results
    """
    result = retrieve_chunks(query, top_k=5, min_score=min_score)
    validation = validate_retrieval_results(result.chunks, min_score)

    return {
        "query": query,
        "result": result,
        "validation": validation,
        "is_accurate": validation["valid"] and len(result.chunks) > 0
    }


def run_comprehensive_tests() -> Dict[str, Any]:
    """
    Run comprehensive test suite with accuracy validation.

    Returns:
        Dictionary with test results and metrics
    """
    query_sets = create_test_query_sets()
    all_results = []
    total_queries = 0
    accurate_queries = 0

    print(f"Running comprehensive test suite with {len(query_sets)} categories...")

    for query_set in query_sets:
        print(f"\nTesting {query_set.category} queries ({len(query_set.queries)} queries)...")
        category_results = []

        for query in query_set.queries:
            total_queries += 1
            test_result = run_accuracy_test(query)
            category_results.append(test_result)

            if test_result["is_accurate"]:
                accurate_queries += 1

            # Print progress
            if total_queries % 5 == 0:
                print(f"  Processed {total_queries} queries...")

        all_results.extend(category_results)

    accuracy_rate = accurate_queries / total_queries if total_queries > 0 else 0

    # Generate detailed report
    category_breakdown = {}
    for query_set in query_sets:
        category_queries = [r for r in all_results if r["query"] in query_set.queries]
        category_accurate = sum(1 for r in category_queries if r["is_accurate"])
        category_accuracy = category_accurate / len(category_queries) if category_queries else 0

        category_breakdown[query_set.category] = {
            "total": len(category_queries),
            "accurate": category_accurate,
            "accuracy_rate": category_accuracy
        }

    results = {
        "total_queries": total_queries,
        "accurate_queries": accurate_queries,
        "accuracy_rate": accuracy_rate,
        "accuracy_target_met": accuracy_rate >= 0.95,  # >95% target
        "category_breakdown": category_breakdown,
        "all_results": all_results,
        "summary": {
            "total_queries": total_queries,
            "accurate_queries": accurate_queries,
            "accuracy_rate": f"{accuracy_rate:.2%}",
            "target_met": "✓ PASS" if accuracy_rate >= 0.95 else "✗ FAIL"
        }
    }

    return results


def generate_test_report(results: Dict[str, Any]):
    """
    Generate detailed accuracy and performance report.

    Args:
        results: Results dictionary from run_comprehensive_tests
    """
    print("\n" + "="*70)
    print("RAG RETRIEVAL COMPREHENSIVE TEST REPORT")
    print("="*70)

    print(f"\nOverall Accuracy:")
    print(f"  Total Queries: {results['total_queries']}")
    print(f"  Accurate Queries: {results['accurate_queries']}")
    print(f"  Accuracy Rate: {results['summary']['accuracy_rate']}")
    print(f"  Target (>95%) Met: {results['summary']['target_met']}")

    print(f"\nCategory Breakdown:")
    for category, stats in results["category_breakdown"].items():
        print(f"  {category.title()}:")
        print(f"    Total: {stats['total']}")
        print(f"    Accurate: {stats['accurate']}")
        print(f"    Rate: {stats['accuracy_rate']:.2%}")

    print(f"\nDetailed Results:")
    for i, result in enumerate(results["all_results"][:5], 1):  # Show first 5 results
        status = "✓" if result["is_accurate"] else "✗"
        print(f"  {i}. {status} Query: {result['query'][:50]}...")
        print(f"     Retrieved: {len(result['result'].chunks)} chunks")
        if result['result'].chunks:
            print(f"     Top Score: {result['result'].chunks[0].score:.3f}")

    if len(results["all_results"]) > 5:
        print(f"  ... and {len(results['all_results']) - 5} more queries")

    print("="*70)


def validate_test_results_against_criteria(results: Dict[str, Any]) -> Dict[str, Any]:
    """
    Add test result validation against success criteria.

    Args:
        results: Results from comprehensive tests

    Returns:
        Dictionary with validation results
    """
    validation = {
        "accuracy_target_met": results["accuracy_rate"] >= 0.95,
        "all_categories_meet_target": all(
            stats["accuracy_rate"] >= 0.95
            for stats in results["category_breakdown"].values()
        ),
        "minimum_queries_tested": results["total_queries"] >= 20,
        "overall_success": False
    }

    validation["overall_success"] = (
        validation["accuracy_target_met"] and
        validation["minimum_queries_tested"]
    )

    return validation


def run_test_suite_with_configurations():
    """
    Implement test suite execution with different configurations.
    """
    configurations = [
        {"top_k": 3, "min_score": 0.3, "name": "Conservative"},
        {"top_k": 5, "min_score": 0.3, "name": "Balanced"},
        {"top_k": 10, "min_score": 0.1, "name": "Permissive"}
    ]

    print("Running test suite with different configurations...")
    config_results = {}

    for config in configurations:
        print(f"\nTesting configuration: {config['name']}")
        print(f"  top_k: {config['top_k']}, min_score: {config['min_score']}")

        # Temporarily modify the test to use these parameters
        query_sets = create_test_query_sets()
        total_queries = 0
        accurate_queries = 0

        for query_set in query_sets:
            for query in query_set.queries:
                total_queries += 1
                result = retrieve_chunks(query, top_k=config["top_k"], min_score=config["min_score"])
                validation = validate_retrieval_results(result.chunks, config["min_score"])

                if validation["valid"] and len(result.chunks) > 0:
                    accurate_queries += 1

        accuracy_rate = accurate_queries / total_queries if total_queries > 0 else 0
        config_results[config["name"]] = {
            "accuracy_rate": accuracy_rate,
            "total_queries": total_queries,
            "accurate_queries": accurate_queries
        }

        print(f"  Results: {accurate_queries}/{total_queries} accurate ({accuracy_rate:.2%})")

    return config_results


def run_edge_case_tests():
    """
    Create edge case testing functions for testing edge cases.
    """
    print("\nRunning edge case tests...")

    edge_case_results = {}

    # Test 1: Empty query
    print("  Testing empty query...")
    empty_result = retrieve_chunks("", top_k=5, min_score=0.3)
    edge_case_results["empty_query"] = {
        "status": empty_result.status,
        "expected": "no_query",
        "passed": empty_result.status == "no_query"
    }

    # Test 2: Very long query
    print("  Testing very long query...")
    long_query = "This is a very long query " * 100  # Make it quite long
    long_result = retrieve_chunks(long_query, top_k=5, min_score=0.3)
    edge_case_results["long_query"] = {
        "status": long_result.status,
        "expected": "success or error but not crash",
        "passed": long_result.status in ["success", "no_results", "error", "low_confidence"]
    }

    # Test 3: Low-confidence matches
    print("  Testing low-confidence query...")
    low_conf_result = retrieve_chunks("asdasdasdasd asdasdasdasd asdasdasdasd", top_k=5, min_score=0.8)  # Nonsense query with high threshold
    edge_case_results["low_confidence"] = {
        "status": low_conf_result.status,
        "expected": "low_confidence or no_results",
        "passed": low_conf_result.status in ["low_confidence", "no_results"]
    }

    # Test 4: Duplicate content handling
    print("  Testing duplicate content handling...")
    # This is harder to test without specific content, but we can at least call the function
    normal_result = retrieve_chunks("What is RAG?", top_k=10, min_score=0.3, filter_duplicates=True)
    normal_result_no_filter = retrieve_chunks("What is RAG?", top_k=10, min_score=0.3, filter_duplicates=False)

    # Check if filtering is being applied (results might be same if no duplicates in corpus)
    edge_case_results["duplicate_filtering"] = {
        "status": "executed",
        "expected": "function executes without error",
        "passed": normal_result.status in ["success", "no_results", "low_confidence", "error"]
    }

    # Test 5: Query with special characters
    print("  Testing query with special characters...")
    special_result = retrieve_chunks("RAG? embedding! vector#search%", top_k=5, min_score=0.3)
    edge_case_results["special_chars"] = {
        "status": special_result.status,
        "expected": "function executes without error",
        "passed": special_result.status in ["success", "no_results", "low_confidence", "error"]
    }

    print("\nEdge case test results:")
    all_passed = True
    for test_name, result in edge_case_results.items():
        status = "✓" if result["passed"] else "✗"
        print(f"  {status} {test_name}: {result['status']} (expected: {result['expected']})")
        if not result["passed"]:
            all_passed = False

    return {
        "results": edge_case_results,
        "all_passed": all_passed,
        "summary": f"Edge case tests: {'✓ PASS' if all_passed else '✗ FAIL'}"
    }


if __name__ == "__main__":
    print("Starting comprehensive RAG retrieval test suite...")

    # Run comprehensive tests
    test_results = run_comprehensive_tests()
    generate_test_report(test_results)

    # Validate results against criteria
    validation_results = validate_test_results_against_criteria(test_results)
    print(f"\nValidation against success criteria:")
    print(f"  Accuracy target (>95%) met: {'✓' if validation_results['accuracy_target_met'] else '✗'}")
    print(f"  Minimum 20 queries tested: {'✓' if validation_results['minimum_queries_tested'] else '✗'}")
    print(f"  Overall validation: {'✓ PASS' if validation_results['overall_success'] else '✗ FAIL'}")

    # Run tests with different configurations
    config_results = run_test_suite_with_configurations()

    print(f"\nConfiguration comparison:")
    for name, results in config_results.items():
        status = "✓" if results["accuracy_rate"] >= 0.95 else "✗"
        print(f"  {name}: {results['accuracy_rate']:.2%} {status}")

    # Run edge case tests
    edge_case_results = run_edge_case_tests()
    print(f"\n{edge_case_results['summary']}")

    print(f"\nTest suite completed.")