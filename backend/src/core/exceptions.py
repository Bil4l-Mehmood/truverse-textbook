"""Custom exception classes for the application."""


class ApplicationError(Exception):
    """Base exception for all application errors."""

    def __init__(self, message: str, code: str = "APPLICATION_ERROR"):
        self.message = message
        self.code = code
        super().__init__(self.message)


class AuthenticationError(ApplicationError):
    """Exception for authentication failures."""

    def __init__(self, message: str = "Authentication failed"):
        super().__init__(message, code="AUTHENTICATION_ERROR")


class ValidationError(ApplicationError):
    """Exception for validation failures."""

    def __init__(self, message: str = "Validation failed"):
        super().__init__(message, code="VALIDATION_ERROR")


class DatabaseError(ApplicationError):
    """Exception for database operation failures."""

    def __init__(self, message: str = "Database operation failed"):
        super().__init__(message, code="DATABASE_ERROR")


class VectorStoreError(ApplicationError):
    """Exception for Qdrant/vector store operation failures."""

    def __init__(self, message: str = "Vector store operation failed"):
        super().__init__(message, code="VECTOR_STORE_ERROR")


class OpenAIError(ApplicationError):
    """Exception for OpenAI API failures."""

    def __init__(self, message: str = "OpenAI API request failed"):
        super().__init__(message, code="OPENAI_ERROR")
