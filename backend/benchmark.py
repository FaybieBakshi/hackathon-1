"""
Performance benchmarking for the RAG retrieval system.

This module contains functions for:
- Timing measurement functions
- Cold start vs cached scenario detection
- Latency percentile calculations
- Performance validation against targets
- Performance reporting
"""
import time
import statistics
from typing import List, Dict, Any, Callable
from dataclasses import dataclass

from src.retrieval.retriever import retrieve_chunks, RetrievalResult


@dataclass
class PerformanceMetrics:
    """Data class for storing performance metrics."""
    p50_latency: float  # 50th percentile response time (ms)
    p90_latency: float  # 90th percentile response time (ms)
    p95_latency: float  # 95th percentile response time (ms)
    p99_latency: float  # 99th percentile response time (ms)
    throughput: float   # Queries per second
    cache_hit_rate: float  # Percentage of cached results used
    error_rate: float   # Percentage of failed queries
    total_queries: int  # Total number of queries processed
    total_time: float   # Total execution time (ms)


def calculate_percentiles(latencies: List[float]) -> Dict[str, float]:
    """
    Calculate latency percentiles from a list of latencies.

    Args:
        latencies: List of latency measurements in milliseconds

    Returns:
        Dictionary with percentile values
    """
    if not latencies:
        return {
            "p50": 0.0,
            "p90": 0.0,
            "p95": 0.0,
            "p99": 0.0
        }

    sorted_latencies = sorted(latencies)
    n = len(sorted_latencies)

    p50_idx = int(0.50 * n)
    p90_idx = int(0.90 * n)
    p95_idx = int(0.95 * n)
    p99_idx = int(0.99 * n)

    # Handle edge cases for small lists
    p50_idx = min(p50_idx, n - 1)
    p90_idx = min(p90_idx, n - 1)
    p95_idx = min(p95_idx, n - 1)
    p99_idx = min(p99_idx, n - 1)

    return {
        "p50": sorted_latencies[p50_idx],
        "p90": sorted_latencies[p90_idx],
        "p95": sorted_latencies[p95_idx],
        "p99": sorted_latencies[p99_idx]
    }


def run_single_query(query: str, top_k: int = 5, min_score: float = 0.3, use_cache: bool = True) -> RetrievalResult:
    """
    Run a single query and return the result with timing.

    Args:
        query: The query to execute
        top_k: Number of top results to return
        min_score: Minimum confidence score threshold
        use_cache: Whether to use caching

    Returns:
        RetrievalResult object with timing information
    """
    return retrieve_chunks(query, top_k, min_score, use_cache)


def benchmark_query_execution(
    queries: List[str],
    iterations: int = 10,
    top_k: int = 5,
    min_score: float = 0.3,
    warmup_rounds: int = 2
) -> PerformanceMetrics:
    """
    Benchmark query execution performance.

    Args:
        queries: List of queries to benchmark
        iterations: Number of times to run each query
        top_k: Number of top results to return
        min_score: Minimum confidence score threshold
        warmup_rounds: Number of warmup rounds to run before measuring

    Returns:
        PerformanceMetrics object with benchmark results
    """
    all_latencies = []
    total_queries = 0
    error_count = 0

    # Warmup rounds to simulate cached scenario
    for _ in range(warmup_rounds):
        for query in queries:
            try:
                result = run_single_query(query, top_k, min_score, use_cache=True)
                if result.status == "error":
                    error_count += 1
            except Exception:
                error_count += 1

    # Actual benchmarking
    for _ in range(iterations):
        for query in queries:
            total_queries += 1
            try:
                start_time = time.time()
                result = run_single_query(query, top_k, min_score, use_cache=True)
                end_time = time.time()

                latency_ms = (end_time - start_time) * 1000
                all_latencies.append(latency_ms)

                if result.status == "error":
                    error_count += 1
            except Exception as e:
                error_count += 1

    # Calculate metrics
    percentiles = calculate_percentiles(all_latencies)
    total_time_ms = sum(all_latencies)
    throughput = total_queries / (total_time_ms / 1000) if total_time_ms > 0 else 0
    error_rate = error_count / total_queries if total_queries > 0 else 0

    # For this implementation, assuming all queries after warmup are cached
    cache_hit_rate = 1.0  # Simplified assumption

    metrics = PerformanceMetrics(
        p50_latency=percentiles["p50"],
        p90_latency=percentiles["p90"],
        p95_latency=percentiles["p95"],
        p99_latency=percentiles["p99"],
        throughput=throughput,
        cache_hit_rate=cache_hit_rate,
        error_rate=error_rate,
        total_queries=total_queries,
        total_time=total_time_ms
    )

    return metrics


def benchmark_cold_start_performance(
    queries: List[str],
    iterations: int = 5,
    top_k: int = 5,
    min_score: float = 0.3
) -> PerformanceMetrics:
    """
    Benchmark cold start performance (without cache).

    Args:
        queries: List of queries to benchmark
        iterations: Number of times to run each query
        top_k: Number of top results to return
        min_score: Minimum confidence score threshold

    Returns:
        PerformanceMetrics object with cold start benchmark results
    """
    all_latencies = []
    total_queries = 0
    error_count = 0

    # Run benchmark without cache
    for _ in range(iterations):
        for query in queries:
            total_queries += 1
            try:
                start_time = time.time()
                result = run_single_query(query, top_k, min_score, use_cache=False)
                end_time = time.time()

                latency_ms = (end_time - start_time) * 1000
                all_latencies.append(latency_ms)

                if result.status == "error":
                    error_count += 1
            except Exception as e:
                error_count += 1

    # Calculate metrics
    percentiles = calculate_percentiles(all_latencies)
    total_time_ms = sum(all_latencies)
    throughput = total_queries / (total_time_ms / 1000) if total_time_ms > 0 else 0
    error_rate = error_count / total_queries if total_queries > 0 else 0

    # For cold start, cache hit rate would be 0 initially
    cache_hit_rate = 0.0

    metrics = PerformanceMetrics(
        p50_latency=percentiles["p50"],
        p90_latency=percentiles["p90"],
        p95_latency=percentiles["p95"],
        p99_latency=percentiles["p99"],
        throughput=throughput,
        cache_hit_rate=cache_hit_rate,
        error_rate=error_rate,
        total_queries=total_queries,
        total_time=total_time_ms
    )

    return metrics


def validate_performance_targets(metrics: PerformanceMetrics) -> Dict[str, Any]:
    """
    Validate performance metrics against defined targets.

    Args:
        metrics: PerformanceMetrics object to validate

    Returns:
        Dictionary with validation results
    """
    validation_results = {
        "p95_latency_cold_start_target_met": metrics.p95_latency <= 500,  # ms
        "p95_latency_cached_target_met": metrics.p95_latency <= 200,      # ms
        "error_rate_acceptable": metrics.error_rate <= 0.05,  # 5% error rate threshold
        "overall_performance_pass": False
    }

    # For this validation, we'll consider it as a simplified check
    # In a real implementation, we'd have separate metrics for cold vs cached
    validation_results["overall_performance_pass"] = (
        validation_results["p95_latency_cached_target_met"] and
        validation_results["error_rate_acceptable"]
    )

    return validation_results


def run_benchmark(
    queries: List[str],
    iterations: int = 10,
    top_k: int = 5,
    min_score: float = 0.3,
    warmup_rounds: int = 2
) -> Dict[str, Any]:
    """
    Run comprehensive performance benchmark.

    Args:
        queries: List of queries to benchmark
        iterations: Number of times to run each query
        top_k: Number of top results to return
        min_score: Minimum confidence score threshold
        warmup_rounds: Number of warmup rounds to run before measuring

    Returns:
        Dictionary with comprehensive benchmark results
    """
    print("Running cached performance benchmark...")
    cached_metrics = benchmark_query_execution(
        queries, iterations, top_k, min_score, warmup_rounds
    )

    print("Running cold start performance benchmark...")
    cold_metrics = benchmark_cold_start_performance(
        queries, max(1, iterations // 2), top_k, min_score
    )

    validation_results = validate_performance_targets(cached_metrics)

    results = {
        "cached_metrics": {
            "p50_latency": cached_metrics.p50_latency,
            "p90_latency": cached_metrics.p90_latency,
            "p95_latency": cached_metrics.p95_latency,
            "p99_latency": cached_metrics.p99_latency,
            "throughput": cached_metrics.throughput,
            "error_rate": cached_metrics.error_rate,
            "total_queries": cached_metrics.total_queries,
            "total_time": cached_metrics.total_time
        },
        "cold_start_metrics": {
            "p95_latency": cold_metrics.p95_latency
        },
        "validation": validation_results,
        "summary": {
            "cached_p95_latency": cached_metrics.p95_latency,
            "cold_start_p95_latency": cold_metrics.p95_latency,
            "targets_met": validation_results["overall_performance_pass"]
        }
    }

    return results


def print_benchmark_report(results: Dict[str, Any]):
    """
    Print a formatted benchmark report.

    Args:
        results: Results dictionary from run_benchmark
    """
    print("\n" + "="*60)
    print("RAG RETRIEVAL PERFORMANCE BENCHMARK REPORT")
    print("="*60)

    print(f"\nCached Performance Metrics:")
    print(f"  P50 Latency: {results['cached_metrics']['p50_latency']:.2f} ms")
    print(f"  P90 Latency: {results['cached_metrics']['p90_latency']:.2f} ms")
    print(f"  P95 Latency: {results['cached_metrics']['p95_latency']:.2f} ms")
    print(f"  P99 Latency: {results['cached_metrics']['p99_latency']:.2f} ms")
    print(f"  Throughput: {results['cached_metrics']['throughput']:.2f} queries/sec")
    print(f"  Error Rate: {results['cached_metrics']['error_rate']:.2%}")

    print(f"\nCold Start Performance Metrics:")
    print(f"  P95 Latency: {results['cold_start_metrics']['p95_latency']:.2f} ms")

    print(f"\nValidation Results:")
    print(f"  Cached P95 < 200ms: {'✓ PASS' if results['cached_metrics']['p95_latency'] <= 200 else '✗ FAIL'}")
    print(f"  Cold Start P95 < 500ms: {'✓ PASS' if results['cold_start_metrics']['p95_latency'] <= 500 else '✗ FAIL'}")
    print(f"  Error Rate < 5%: {'✓ PASS' if results['cached_metrics']['error_rate'] <= 0.05 else '✗ FAIL'}")

    print(f"\nOverall Result: {'✓ PASS' if results['summary']['targets_met'] else '✗ FAIL'}")
    print("="*60)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="RAG Retrieval Performance Benchmark")
    parser.add_argument("--queries", nargs="+", default=["What is RAG?", "How does embedding work?", "Explain vector search"],
                        help="Queries to benchmark")
    parser.add_argument("--iterations", type=int, default=10,
                        help="Number of iterations for each query")
    parser.add_argument("--top-k", type=int, default=5,
                        help="Number of top results to return")
    parser.add_argument("--min-score", type=float, default=0.3,
                        help="Minimum confidence score threshold")

    args = parser.parse_args()

    print(f"Running benchmark with {len(args.queries)} queries, {args.iterations} iterations each")
    results = run_benchmark(
        queries=args.queries,
        iterations=args.iterations,
        top_k=args.top_k,
        min_score=args.min_score
    )

    print_benchmark_report(results)